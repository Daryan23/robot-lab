import argparse
import math
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.utils import get_schedule_fn
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

from robot_lab_rl.envs.box_push_env import DIFFICULTIES
from robot_lab_rl.rl import (
    DEFAULT_LOG_DIR,
    DEFAULT_MODEL_DIR,
    DEFAULT_RESIDUAL_SCALE,
    make_box_push_env,
    zero_init_residual_policy,
)


class BoxPushMetricsCallback(BaseCallback):
    """Record task-specific learning signals for TensorBoard.

    Logs rolling means over the last ``window`` steps/episodes:

    - ``box_push/*``      task-quality signals plus the episode-outcome split
      (success / no-progress stall / timeout)
    - ``reward_terms/*``  every component of the shaped reward separately, so it
      is visible which terms actually drive learning (and reward hacking)
    - ``residual/*``      how much the policy corrects the scripted expert
      (mean |correction| overall and per wheel, plus saturation) -- only present
      when training in ``--residual`` mode
    """

    SCALAR_INFO_KEYS = (
        "box_zone_distance",
        "push_setup_distance",
        "robot_box_distance",
        "team_robot_box_distance",
        "max_robot_box_distance",
        "heading_alignment",
        "behind_box_quality",
        "contact_quality",
        "contact_balance",
        "complementary_contact_quality",
        "orientation_error",
        "box_velocity_to_zone",
        "box_zone_fit_error",
    )
    BOOL_INFO_KEYS = ("both_robot_contact",)
    RESIDUAL_INFO_KEYS = ("correction_magnitude", "correction_saturation")

    def __init__(self, window: int = 100) -> None:
        super().__init__()
        self.window = window
        self.step_in_zone: deque[float] = deque(maxlen=window)
        self.episode_successes: deque[float] = deque(maxlen=window)
        self.episode_outcomes: deque[str] = deque(maxlen=window)
        self.scalars: dict[str, deque[float]] = defaultdict(lambda: deque(maxlen=window))
        self.reward_terms: dict[str, deque[float]] = defaultdict(lambda: deque(maxlen=window))
        self.residuals: dict[str, deque[float]] = defaultdict(lambda: deque(maxlen=window))

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [False] * len(infos))
        for info, done in zip(infos, dones):
            if "box_in_zone" not in info:
                continue
            self.step_in_zone.append(float(info["box_in_zone"]))
            for key in self.SCALAR_INFO_KEYS:
                value = info.get(key)
                if value is not None:
                    self.scalars[key].append(float(value))
            for key in self.BOOL_INFO_KEYS:
                if key in info:
                    self.scalars[key].append(float(bool(info[key])))
            for key in self.RESIDUAL_INFO_KEYS:
                if key in info:
                    self.residuals[key].append(float(info[key]))
            applied_correction = info.get("applied_correction")
            if applied_correction is not None:
                wheels = np.abs(np.asarray(applied_correction, dtype=np.float32)).reshape(-1)
                for wheel_index, value in enumerate(wheels[:4]):
                    self.residuals[f"correction_wheel{wheel_index}"].append(float(value))
            reward_terms = info.get("reward_terms")
            if isinstance(reward_terms, dict):
                for name, value in reward_terms.items():
                    self.reward_terms[name].append(float(value))
            if done:
                success = bool(info["box_in_zone"])
                self.episode_successes.append(float(success))
                if success:
                    self.episode_outcomes.append("success")
                elif info.get("no_progress", False):
                    self.episode_outcomes.append("stall")
                else:
                    self.episode_outcomes.append("timeout")

        self._record()
        return True

    def _record(self) -> None:
        if self.step_in_zone:
            self.logger.record("box_push/step_in_zone_rate_100", float(np.mean(self.step_in_zone)))
        if self.episode_successes:
            success_rate = float(np.mean(self.episode_successes))
            self.logger.record("box_push/success_rate_100", success_rate)
            self.logger.record("box_push/episode_success_rate_100", success_rate)
        if self.episode_outcomes:
            total = len(self.episode_outcomes)
            for outcome in ("success", "stall", "timeout"):
                fraction = sum(item == outcome for item in self.episode_outcomes) / total
                self.logger.record(f"box_push/outcome_{outcome}_rate_100", fraction)
        for key, values in self.scalars.items():
            if values:
                self.logger.record(f"box_push/{key}_100", float(np.mean(values)))
        for name, values in self.reward_terms.items():
            if values:
                self.logger.record(f"reward_terms/{name}_100", float(np.mean(values)))
        for name, values in self.residuals.items():
            if values:
                self.logger.record(f"residual/{name}_100", float(np.mean(values)))
        return True


class TaskEvalCallback(BaseCallback):
    """Evaluate the current policy on a named task during training."""

    def __init__(
        self,
        difficulty: str,
        label: str,
        eval_every: int,
        episodes: int,
        model_dir: Path,
        max_episode_steps: int,
        best_model_name: str | None = None,
        save_best: bool = False,
        residual: bool = False,
        residual_scale: float = DEFAULT_RESIDUAL_SCALE,
    ) -> None:
        super().__init__()
        self.difficulty = difficulty
        self.label = label
        self.eval_every = eval_every
        self.episodes = episodes
        self.model_dir = model_dir
        self.max_episode_steps = max_episode_steps
        self.best_model_name = best_model_name
        self.save_best = save_best
        self.residual = residual
        self.residual_scale = residual_scale
        self.last_eval_step = 0
        self.best_success_rate = -math.inf
        self.best_average_return = -math.inf

    def _on_step(self) -> bool:
        if self.eval_every <= 0 or self.num_timesteps - self.last_eval_step < self.eval_every:
            return True

        self.last_eval_step = self.num_timesteps
        success_rate, average_return, average_distance, average_fit_error = self._evaluate()
        self.logger.record(f"eval_{self.label}/success_rate", success_rate)
        self.logger.record(f"eval_{self.label}/average_return", average_return)
        self.logger.record(f"eval_{self.label}/average_final_distance", average_distance)
        self.logger.record(f"eval_{self.label}/average_fit_error", average_fit_error)
        print(
            f"Eval {self.label} ({self.difficulty}) at {self.num_timesteps} steps: "
            f"success={success_rate:.1%}, return={average_return:.2f}, "
            f"final_distance={average_distance:.3f} m, fit_error={average_fit_error:.3f} m"
        )

        if self.save_best and (
            success_rate > self.best_success_rate
            or success_rate == self.best_success_rate
            and average_return > self.best_average_return
        ):
            self.best_success_rate = success_rate
            self.best_average_return = average_return
            if self.best_model_name is None:
                raise RuntimeError("best_model_name must be set when save_best=True")
            best_path = self.model_dir / self.best_model_name
            self.model.save(best_path)
            print(
                f"Saved new best {self.label} model to {best_path}.zip "
                f"(success={success_rate:.1%}, return={average_return:.2f})"
            )
        return True

    def _evaluate(self) -> tuple[float, float, float, float]:
        env = make_box_push_env(
            difficulty=self.difficulty,
            max_steps=self.max_episode_steps,
            residual=self.residual,
            residual_scale=self.residual_scale,
        )
        successes = 0
        returns: list[float] = []
        final_distances: list[float] = []
        final_fit_errors: list[float] = []
        try:
            for _ in range(self.episodes):
                observation, _ = env.reset()
                episode_return = 0.0
                info = {}
                for _ in range(env.unwrapped.max_steps):
                    action, _ = self.model.predict(observation, deterministic=True)
                    observation, reward, terminated, truncated, info = env.step(action)
                    episode_return += float(reward)
                    if terminated or truncated:
                        break
                successes += int(info.get("box_in_zone", False))
                returns.append(episode_return)
                final_distances.append(float(info.get("box_zone_distance", np.nan)))
                final_fit_errors.append(float(info.get("box_zone_fit_error", np.nan)))
        finally:
            env.close()

        return (
            successes / self.episodes,
            float(np.mean(returns)),
            float(np.nanmean(final_distances)),
            float(np.nanmean(final_fit_errors)),
        )


def make_training_env(
    difficulty: str,
    num_envs: int,
    log_dir: Path,
    max_episode_steps: int,
    randomization: float,
    residual: bool = False,
    residual_scale: float = DEFAULT_RESIDUAL_SCALE,
) -> VecMonitor:
    env = DummyVecEnv(
        [
            lambda difficulty=difficulty: make_box_push_env(
                difficulty=difficulty,
                max_steps=max_episode_steps,
                randomization=randomization,
                residual=residual,
                residual_scale=residual_scale,
            )
            for _ in range(num_envs)
        ]
    )
    return VecMonitor(env, filename=str(log_dir / f"monitor_{difficulty}.csv"))


def curriculum_phases(total_timesteps: int) -> list[tuple[str, int]]:
    easy_steps = int(total_timesteps * 0.25)
    medium_steps = int(total_timesteps * 0.35)
    full_steps = total_timesteps - easy_steps - medium_steps
    return [("easy", easy_steps), ("medium", medium_steps), ("full", full_steps)]


def evaluate_model_on_difficulty(
    model: PPO,
    difficulty: str,
    episodes: int,
    max_episode_steps: int,
    residual: bool = False,
    residual_scale: float = DEFAULT_RESIDUAL_SCALE,
) -> tuple[float, float, float]:
    env = make_box_push_env(
        difficulty=difficulty,
        max_steps=max_episode_steps,
        residual=residual,
        residual_scale=residual_scale,
    )
    successes = 0
    returns: list[float] = []
    fit_errors: list[float] = []
    try:
        for _ in range(episodes):
            observation, _ = env.reset()
            episode_return = 0.0
            info = {}
            for _ in range(env.unwrapped.max_steps):
                action, _ = model.predict(observation, deterministic=True)
                observation, reward, terminated, truncated, info = env.step(action)
                episode_return += float(reward)
                if terminated or truncated:
                    break
            successes += int(info.get("box_in_zone", False))
            returns.append(episode_return)
            fit_errors.append(float(info.get("box_zone_fit_error", np.nan)))
    finally:
        env.close()
    return successes / episodes, float(np.mean(returns)), float(np.nanmean(fit_errors))


def apply_ppo_hyperparameters(
    model: PPO,
    learning_rate: float,
    ent_coef: float,
    clip_range: float,
    target_kl: float | None,
) -> None:
    """Override PPO update settings after loading a pretrained model."""

    model.learning_rate = learning_rate
    model.lr_schedule = get_schedule_fn(learning_rate)
    model.ent_coef = ent_coef
    model.clip_range = get_schedule_fn(clip_range)
    model.target_kl = target_kl


def main() -> None:
    parser = argparse.ArgumentParser(description="Train PPO on the box-push task.")
    parser.add_argument("--timesteps", type=int, default=2_000_000, help="Total PPO training timesteps.")
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR, help="Directory for saved models.")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR, help="TensorBoard and monitor log directory.")
    parser.add_argument("--checkpoint-every", type=int, default=100_000, help="Checkpoint frequency in timesteps.")
    parser.add_argument("--eval-every", type=int, default=50_000, help="Full-task evaluation frequency in timesteps.")
    parser.add_argument("--eval-episodes", type=int, default=10, help="Episodes per full-task evaluation.")
    parser.add_argument(
        "--no-full-eval",
        action="store_true",
        help=(
            "Skip the separate full-task evaluation/logging (eval_full/*). Useful when training a "
            "single non-full difficulty such as coop_random, where the full task is not relevant."
        ),
    )
    parser.add_argument("--difficulty", choices=("curriculum", *DIFFICULTIES), default="curriculum")
    parser.add_argument("--num-envs", type=int, default=8, help="Parallel training environments.")
    parser.add_argument("--n-steps", type=int, default=2048, help="PPO rollout length before each update.")
    parser.add_argument("--batch-size", type=int, default=512, help="PPO minibatch size.")
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument("--randomization", type=float, default=0.0, help="Training layout randomization radius.")
    parser.add_argument(
        "--residual",
        action="store_true",
        help=(
            "Residual mode: keep the scripted expert in the control loop and let the policy "
            "learn only a bounded additive correction (action = clip(expert + scale*policy))."
        ),
    )
    parser.add_argument(
        "--residual-scale",
        type=float,
        default=DEFAULT_RESIDUAL_SCALE,
        help="Bound on the policy correction added on top of the expert action (residual mode).",
    )
    parser.add_argument(
        "--residual-log-std-init",
        type=float,
        default=-1.0,
        help="Initial action log-std for the residual policy (lower = gentler initial exploration).",
    )
    parser.add_argument("--learning-rate", type=float, default=2.5e-4)
    parser.add_argument("--ent-coef", type=float, default=0.01)
    parser.add_argument("--clip-range", type=float, default=0.2)
    parser.add_argument("--target-kl", type=float, default=None)
    parser.add_argument(
        "--conservative-finetune",
        action="store_true",
        help=(
            "Use small PPO updates for pretrained policies "
            "(learning_rate=3e-5, ent_coef=0, clip_range=0.05, target_kl=0.01)."
        ),
    )
    parser.add_argument("--pretrained-model", type=Path, default=None, help="Optional BC/pretrained PPO .zip model.")
    parser.add_argument("--pretrained-gate-episodes", type=int, default=20)
    parser.add_argument("--pretrained-gate-difficulty", choices=DIFFICULTIES, default="easy")
    parser.add_argument("--min-pretrained-success", type=float, default=0.80)
    parser.add_argument(
        "--skip-pretrained-gate",
        action="store_true",
        help="Allow PPO fine-tuning even if the pretrained model fails the easy-task gate.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for PPO. Use different seeds across runs to produce error-band figures.",
    )
    args = parser.parse_args()

    if args.conservative_finetune:
        args.learning_rate = 3e-5
        args.ent_coef = 0.0
        args.clip_range = 0.05
        args.target_kl = 0.01

    args.model_dir.mkdir(parents=True, exist_ok=True)
    args.log_dir.mkdir(parents=True, exist_ok=True)

    phases = (
        curriculum_phases(args.timesteps)
        if args.difficulty == "curriculum"
        else [(args.difficulty, args.timesteps)]
    )
    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.checkpoint_every // args.num_envs, 1),
        save_path=str(args.model_dir),
        name_prefix="box_push_ppo",
    )
    full_eval_callback = None
    if not args.no_full_eval:
        full_eval_callback = TaskEvalCallback(
            difficulty="full",
            label="full",
            eval_every=args.eval_every,
            episodes=args.eval_episodes,
            model_dir=args.model_dir,
            max_episode_steps=args.max_episode_steps,
            best_model_name="box_push_ppo_best_full",
            save_best=True,
            residual=args.residual,
            residual_scale=args.residual_scale,
        )

    model: PPO | None = None
    env = None
    for difficulty, phase_steps in phases:
        if phase_steps <= 0:
            continue
        print(f"Training phase: {difficulty} for {phase_steps} timesteps")
        if env is not None:
            env.close()
        env = make_training_env(
            difficulty,
            args.num_envs,
            args.log_dir,
            args.max_episode_steps,
            args.randomization,
            residual=args.residual,
            residual_scale=args.residual_scale,
        )
        if model is None:
            if args.residual:
                model = PPO(
                    "MlpPolicy",
                    env,
                    verbose=1,
                    tensorboard_log=str(args.log_dir),
                    n_steps=args.n_steps,
                    batch_size=args.batch_size,
                    gamma=0.995,
                    gae_lambda=0.95,
                    ent_coef=args.ent_coef,
                    learning_rate=args.learning_rate,
                    clip_range=args.clip_range,
                    target_kl=args.target_kl,
                    max_grad_norm=0.5,
                    seed=args.seed,
                    policy_kwargs={
                        "net_arch": [128, 128],
                        "log_std_init": args.residual_log_std_init,
                    },
                )
                zero_init_residual_policy(model)
                if args.pretrained_model is not None:
                    print(
                        "Residual mode: ignoring --pretrained-model and starting from a "
                        "zero-init correction policy (action starts equal to the expert)."
                    )
                print(
                    "Residual mode: the scripted expert stays in the control loop; the policy "
                    f"learns a bounded correction (scale={args.residual_scale})."
                )
            elif args.pretrained_model is not None:
                model = PPO.load(args.pretrained_model, env=env, tensorboard_log=str(args.log_dir))
                model.verbose = 1
                apply_ppo_hyperparameters(
                    model,
                    learning_rate=args.learning_rate,
                    ent_coef=args.ent_coef,
                    clip_range=args.clip_range,
                    target_kl=args.target_kl,
                )
                print(f"Loaded pretrained model: {args.pretrained_model}")
                if not args.skip_pretrained_gate:
                    success_rate, average_return, average_fit_error = evaluate_model_on_difficulty(
                        model,
                        args.pretrained_gate_difficulty,
                        args.pretrained_gate_episodes,
                        args.max_episode_steps,
                    )
                    print(
                        f"Pretrained gate on {args.pretrained_gate_difficulty}: "
                        f"success={success_rate:.1%}, return={average_return:.2f}, "
                        f"fit_error={average_fit_error:.3f} m"
                    )
                    if success_rate < args.min_pretrained_success:
                        raise SystemExit(
                            "Pretrained gate failed. Do not fine-tune this model yet. "
                            "Run scripts/build_box_push_prior.py until it passes, or pass "
                            "--skip-pretrained-gate only for debugging."
                        )
            else:
                model = PPO(
                    "MlpPolicy",
                    env,
                    verbose=1,
                    tensorboard_log=str(args.log_dir),
                    n_steps=args.n_steps,
                    batch_size=args.batch_size,
                    gamma=0.995,
                    gae_lambda=0.95,
                    ent_coef=args.ent_coef,
                    learning_rate=args.learning_rate,
                    clip_range=args.clip_range,
                    target_kl=args.target_kl,
                    max_grad_norm=0.5,
                    seed=args.seed,
                    policy_kwargs={"net_arch": [128, 128]},
                )
        else:
            model.set_env(env)

        phase_callbacks = [
            checkpoint_callback,
            BoxPushMetricsCallback(),
            TaskEvalCallback(
                difficulty=difficulty,
                label=f"current_{difficulty}",
                eval_every=args.eval_every,
                episodes=args.eval_episodes,
                model_dir=args.model_dir,
                max_episode_steps=args.max_episode_steps,
                best_model_name=f"box_push_ppo_best_current_{difficulty}",
                save_best=True,
                residual=args.residual,
                residual_scale=args.residual_scale,
            ),
        ]
        if full_eval_callback is not None:
            phase_callbacks.append(full_eval_callback)
        callbacks = CallbackList(phase_callbacks)
        model.learn(
            total_timesteps=phase_steps,
            callback=callbacks,
            tb_log_name="ppo",
            progress_bar=False,
            reset_num_timesteps=False,
        )
        model.save(args.model_dir / f"box_push_ppo_after_{difficulty}")

    assert model is not None
    final_path = args.model_dir / "box_push_ppo_final"
    model.save(final_path)
    if env is not None:
        env.close()
    print(f"Saved final model to {final_path}.zip")


if __name__ == "__main__":
    main()
