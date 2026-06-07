import argparse
import math
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.utils import get_schedule_fn
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

from robot_lab_rl.envs.box_push_env import DIFFICULTIES
from robot_lab_rl.rl import DEFAULT_LOG_DIR, DEFAULT_MODEL_DIR, make_box_push_env


class BoxPushMetricsCallback(BaseCallback):
    """Record task-specific learning signals for TensorBoard."""

    def __init__(self) -> None:
        super().__init__()
        self.episode_successes: list[float] = []
        self.step_in_zone: list[float] = []
        self.distances: list[float] = []
        self.push_setup_distances: list[float] = []
        self.robot_box_distances: list[float] = []
        self.heading_alignments: list[float] = []
        self.behind_box_qualities: list[float] = []
        self.contact_qualities: list[float] = []

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [False] * len(infos))
        for info, done in zip(infos, dones):
            if "box_in_zone" in info:
                self.step_in_zone.append(float(info["box_in_zone"]))
                if done:
                    self.episode_successes.append(float(info["box_in_zone"]))
            if "box_zone_distance" in info:
                self.distances.append(float(info["box_zone_distance"]))
            if "push_setup_distance" in info:
                self.push_setup_distances.append(float(info["push_setup_distance"]))
            if "robot_box_distance" in info:
                self.robot_box_distances.append(float(info["robot_box_distance"]))
            if "heading_alignment" in info:
                self.heading_alignments.append(float(info["heading_alignment"]))
            if "behind_box_quality" in info:
                self.behind_box_qualities.append(float(info["behind_box_quality"]))
            if "contact_quality" in info:
                self.contact_qualities.append(float(info["contact_quality"]))

        if self.episode_successes:
            recent_success_rate = np.mean(self.episode_successes[-100:])
            self.logger.record("box_push/success_rate_100", recent_success_rate)
            self.logger.record("box_push/episode_success_rate_100", recent_success_rate)
        if self.step_in_zone:
            self.logger.record("box_push/step_in_zone_rate_100", np.mean(self.step_in_zone[-100:]))
        if self.distances:
            self.logger.record("box_push/box_zone_distance_100", np.mean(self.distances[-100:]))
        if self.push_setup_distances:
            self.logger.record("box_push/push_setup_distance_100", np.mean(self.push_setup_distances[-100:]))
        if self.robot_box_distances:
            self.logger.record("box_push/robot_box_distance_100", np.mean(self.robot_box_distances[-100:]))
        if self.heading_alignments:
            self.logger.record("box_push/heading_alignment_100", np.mean(self.heading_alignments[-100:]))
        if self.behind_box_qualities:
            self.logger.record("box_push/behind_box_quality_100", np.mean(self.behind_box_qualities[-100:]))
        if self.contact_qualities:
            self.logger.record("box_push/contact_quality_100", np.mean(self.contact_qualities[-100:]))
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
        env = make_box_push_env(difficulty=self.difficulty, max_steps=self.max_episode_steps)
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
) -> VecMonitor:
    env = DummyVecEnv(
        [
            lambda difficulty=difficulty: make_box_push_env(
                difficulty=difficulty,
                max_steps=max_episode_steps,
                randomization=randomization,
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
) -> tuple[float, float, float]:
    env = make_box_push_env(difficulty=difficulty, max_steps=max_episode_steps)
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
    parser.add_argument("--difficulty", choices=("curriculum", *DIFFICULTIES), default="curriculum")
    parser.add_argument("--num-envs", type=int, default=8, help="Parallel training environments.")
    parser.add_argument("--n-steps", type=int, default=2048, help="PPO rollout length before each update.")
    parser.add_argument("--batch-size", type=int, default=512, help="PPO minibatch size.")
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument("--randomization", type=float, default=0.0, help="Training layout randomization radius.")
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
    full_eval_callback = TaskEvalCallback(
        difficulty="full",
        label="full",
        eval_every=args.eval_every,
        episodes=args.eval_episodes,
        model_dir=args.model_dir,
        max_episode_steps=args.max_episode_steps,
        best_model_name="box_push_ppo_best_full",
        save_best=True,
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
        )
        if model is None:
            if args.pretrained_model is not None:
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
                    policy_kwargs={"net_arch": [128, 128]},
                )
        else:
            model.set_env(env)

        callbacks = CallbackList(
            [
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
                ),
                full_eval_callback,
            ]
        )
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
