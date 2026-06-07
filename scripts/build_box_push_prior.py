import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from torch.utils.tensorboard import SummaryWriter

from robot_lab_rl import BoxPushEnv, BoxPushHeuristicPolicy
from robot_lab_rl.envs.box_push_env import DIFFICULTIES
from robot_lab_rl.rl import DEFAULT_LOG_DIR, DEFAULT_MODEL_DIR, FlatActionWrapper
try:
    from pretrain_box_push_bc import make_bc_model, policy_action_mean
except ModuleNotFoundError:
    from scripts.pretrain_box_push_bc import make_bc_model, policy_action_mean


@dataclass(frozen=True)
class EvalResult:
    success_rate: float
    average_return: float
    average_length: float
    average_distance: float
    average_fit_error: float


def eval_is_better(candidate: EvalResult, incumbent: EvalResult | None) -> bool:
    if incumbent is None:
        return True
    candidate_key = (
        candidate.success_rate,
        -candidate.average_fit_error,
        candidate.average_return,
        -candidate.average_length,
    )
    incumbent_key = (
        incumbent.success_rate,
        -incumbent.average_fit_error,
        incumbent.average_return,
        -incumbent.average_length,
    )
    return candidate_key > incumbent_key


def evaluate_controller(
    controller,
    *,
    difficulty: str,
    episodes: int,
    max_episode_steps: int,
    randomization: float,
) -> EvalResult:
    env = FlatActionWrapper(
        BoxPushEnv(
            render_mode="direct",
            difficulty=difficulty,
            max_steps=max_episode_steps,
            randomization=randomization,
        )
    )
    successes = 0
    returns: list[float] = []
    lengths: list[int] = []
    distances: list[float] = []
    fit_errors: list[float] = []
    try:
        for _ in range(episodes):
            observation, _ = env.reset()
            episode_return = 0.0
            info = {}
            for step in range(max_episode_steps):
                if isinstance(controller, PPO):
                    action, _ = controller.predict(observation, deterministic=True)
                else:
                    action = controller.predict(observation)
                observation, reward, terminated, truncated, info = env.step(action)
                episode_return += float(reward)
                if terminated or truncated:
                    break

            successes += int(info.get("box_in_zone", False))
            returns.append(episode_return)
            lengths.append(step + 1)
            distances.append(float(info.get("box_zone_distance", np.nan)))
            fit_errors.append(float(info.get("box_zone_fit_error", np.nan)))
    finally:
        env.close()

    return EvalResult(
        success_rate=successes / max(episodes, 1),
        average_return=float(np.mean(returns)),
        average_length=float(np.mean(lengths)),
        average_distance=float(np.nanmean(distances)),
        average_fit_error=float(np.nanmean(fit_errors)),
    )


def print_eval(label: str, result: EvalResult) -> None:
    print(
        f"{label}: success={result.success_rate:.1%}, "
        f"return={result.average_return:.2f}, "
        f"length={result.average_length:.1f}, "
        f"distance={result.average_distance:.3f} m, "
        f"fit_error={result.average_fit_error:.3f} m"
    )


def collect_expert_dataset(
    *,
    expert: BoxPushHeuristicPolicy,
    difficulty: str,
    target_steps: int,
    max_episode_steps: int,
    randomization: float,
    max_attempts: int,
) -> tuple[np.ndarray, np.ndarray]:
    env = FlatActionWrapper(
        BoxPushEnv(
            render_mode="direct",
            difficulty=difficulty,
            max_steps=max_episode_steps,
            randomization=randomization,
        )
    )
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    attempts = 0
    successes = 0
    try:
        while len(observations) < target_steps and attempts < max_attempts:
            attempts += 1
            observation, _ = env.reset()
            episode_observations: list[np.ndarray] = []
            episode_actions: list[np.ndarray] = []
            info = {}
            for _ in range(max_episode_steps):
                action = expert.predict(observation)
                next_observation, _, terminated, truncated, info = env.step(action)
                episode_observations.append(observation.copy())
                episode_actions.append(action.copy())
                observation = next_observation
                if terminated or truncated:
                    break

            if info.get("box_in_zone", False):
                successes += 1
                remaining = target_steps - len(observations)
                observations.extend(episode_observations[:remaining])
                actions.extend(episode_actions[:remaining])
            print(
                f"seed demos: collected={len(observations)}/{target_steps}, "
                f"attempt={attempts}, success={bool(info.get('box_in_zone', False))}"
            )
    finally:
        env.close()

    if not observations:
        raise RuntimeError("The expert did not produce any successful demonstration episodes.")
    print(f"seed demos: kept {successes} successful episodes from {attempts} attempts")
    return np.asarray(observations, dtype=np.float32), np.asarray(actions, dtype=np.float32)


def collect_dagger_labels(
    *,
    model: PPO,
    expert: BoxPushHeuristicPolicy,
    difficulty: str,
    episodes: int,
    max_episode_steps: int,
    randomization: float,
    expert_action_probability: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, EvalResult]:
    env = FlatActionWrapper(
        BoxPushEnv(
            render_mode="direct",
            difficulty=difficulty,
            max_steps=max_episode_steps,
            randomization=randomization,
        )
    )
    observations: list[np.ndarray] = []
    labels: list[np.ndarray] = []
    successes = 0
    returns: list[float] = []
    lengths: list[int] = []
    distances: list[float] = []
    fit_errors: list[float] = []
    try:
        for _ in range(episodes):
            observation, _ = env.reset()
            episode_return = 0.0
            info = {}
            for step in range(max_episode_steps):
                expert_action = expert.predict(observation)
                if rng.random() < expert_action_probability:
                    action = expert_action
                else:
                    action, _ = model.predict(observation, deterministic=True)
                observations.append(observation.copy())
                labels.append(expert_action.copy())
                observation, reward, terminated, truncated, info = env.step(action)
                episode_return += float(reward)
                if terminated or truncated:
                    break

            successes += int(info.get("box_in_zone", False))
            returns.append(episode_return)
            lengths.append(step + 1)
            distances.append(float(info.get("box_zone_distance", np.nan)))
            fit_errors.append(float(info.get("box_zone_fit_error", np.nan)))
    finally:
        env.close()

    result = EvalResult(
        success_rate=successes / max(episodes, 1),
        average_return=float(np.mean(returns)),
        average_length=float(np.mean(lengths)),
        average_distance=float(np.nanmean(distances)),
        average_fit_error=float(np.nanmean(fit_errors)),
    )
    return np.asarray(observations, dtype=np.float32), np.asarray(labels, dtype=np.float32), result


def train_supervised(
    *,
    model: PPO,
    observations: np.ndarray,
    actions: np.ndarray,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    writer: SummaryWriter,
    global_epoch_offset: int,
) -> float:
    model.policy.set_training_mode(True)
    optimizer = torch.optim.Adam(model.policy.parameters(), lr=learning_rate)
    device = model.device
    observations_tensor = torch.as_tensor(observations, device=device)
    actions_tensor = torch.as_tensor(actions, device=device)
    rng = np.random.default_rng(11 + global_epoch_offset)
    last_loss = 0.0
    for epoch in range(1, epochs + 1):
        indices = rng.permutation(len(observations))
        losses: list[float] = []
        for start in range(0, len(indices), batch_size):
            batch_indices = indices[start : start + batch_size]
            predicted_actions = policy_action_mean(model, observations_tensor[batch_indices])
            loss = torch.nn.functional.mse_loss(predicted_actions, actions_tensor[batch_indices])
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.policy.parameters(), 0.5)
            optimizer.step()
            losses.append(float(loss.detach().cpu()))
        last_loss = float(np.mean(losses))
        writer.add_scalar("prior/supervised_loss", last_loss, global_epoch_offset + epoch)
        print(f"supervised epoch={global_epoch_offset + epoch:03d} loss={last_loss:.6f}")
    return last_loss


def downsample_dataset(
    observations: np.ndarray,
    actions: np.ndarray,
    *,
    max_steps: int,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray]:
    if len(observations) <= max_steps:
        return observations, actions
    indices = rng.choice(len(observations), size=max_steps, replace=False)
    return observations[indices], actions[indices]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build a gated box-push prior with expert demos, BC, and DAgger correction."
    )
    parser.add_argument("--difficulty", choices=DIFFICULTIES, default="easy")
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR.parent / "box_push_prior")
    parser.add_argument(
        "--initial-model",
        type=Path,
        default=None,
        help="Optional PPO .zip to continue from when building a harder-stage prior.",
    )
    parser.add_argument(
        "--output-name",
        default="box_push_prior",
        help="Base filename for the saved prior inside --model-dir.",
    )
    parser.add_argument("--seed-demo-steps", type=int, default=30_000)
    parser.add_argument("--seed-max-attempts", type=int, default=500)
    parser.add_argument("--dagger-iterations", type=int, default=6)
    parser.add_argument("--dagger-episodes", type=int, default=20)
    parser.add_argument("--epochs-per-iteration", type=int, default=12)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--max-dataset-steps", type=int, default=150_000)
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument("--randomization", type=float, default=0.015)
    parser.add_argument("--eval-episodes", type=int, default=20)
    parser.add_argument("--min-success-rate", type=float, default=0.80)
    parser.add_argument(
        "--save-best",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Save the best evaluated prior as <output-name>_best.zip.",
    )
    args = parser.parse_args()

    args.model_dir.mkdir(parents=True, exist_ok=True)
    args.log_dir.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(23)
    expert = BoxPushHeuristicPolicy()
    expert_eval = evaluate_controller(
        expert,
        difficulty=args.difficulty,
        episodes=args.eval_episodes,
        max_episode_steps=args.max_episode_steps,
        randomization=args.randomization,
    )
    print_eval("expert gate", expert_eval)
    if expert_eval.success_rate < args.min_success_rate:
        raise SystemExit(
            f"Expert gate failed: expected at least {args.min_success_rate:.0%} success. "
            "Fix the heuristic before training a neural prior."
        )

    observations, actions = collect_expert_dataset(
        expert=expert,
        difficulty=args.difficulty,
        target_steps=args.seed_demo_steps,
        max_episode_steps=args.max_episode_steps,
        randomization=args.randomization,
        max_attempts=args.seed_max_attempts,
    )

    writer = SummaryWriter(str(args.log_dir))
    if args.initial_model is not None:
        env_for_loaded_model = make_bc_model(difficulty=args.difficulty, max_steps=args.max_episode_steps).env
        model = PPO.load(args.initial_model, env=env_for_loaded_model)
        print(f"Loaded initial model for staged prior: {args.initial_model}")
    else:
        model = make_bc_model(difficulty=args.difficulty, max_steps=args.max_episode_steps)
    best_eval: EvalResult | None = None
    best_path = args.model_dir / f"{args.output_name}_best"
    global_epoch = 0
    train_supervised(
        model=model,
        observations=observations,
        actions=actions,
        epochs=args.epochs_per_iteration,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        writer=writer,
        global_epoch_offset=global_epoch,
    )
    global_epoch += args.epochs_per_iteration
    model.save(args.model_dir / "prior_seed_bc")
    seed_eval = evaluate_controller(
        model,
        difficulty=args.difficulty,
        episodes=args.eval_episodes,
        max_episode_steps=args.max_episode_steps,
        randomization=args.randomization,
    )
    print_eval("seed BC eval", seed_eval)
    if args.save_best and eval_is_better(seed_eval, best_eval):
        best_eval = seed_eval
        model.save(best_path)
        print(f"Saved best prior so far to {best_path.with_suffix('.zip')}")

    for iteration in range(1, args.dagger_iterations + 1):
        expert_probability = max(0.10, 0.55 * (1.0 - iteration / max(args.dagger_iterations, 1)))
        new_observations, new_actions, rollout_eval = collect_dagger_labels(
            model=model,
            expert=expert,
            difficulty=args.difficulty,
            episodes=args.dagger_episodes,
            max_episode_steps=args.max_episode_steps,
            randomization=args.randomization,
            expert_action_probability=expert_probability,
            rng=rng,
        )
        print_eval(f"dagger rollout {iteration}", rollout_eval)
        observations = np.concatenate([observations, new_observations], axis=0)
        actions = np.concatenate([actions, new_actions], axis=0)
        observations, actions = downsample_dataset(
            observations,
            actions,
            max_steps=args.max_dataset_steps,
            rng=rng,
        )
        print(f"dataset size after dagger {iteration}: {len(observations)} labeled states")
        train_supervised(
            model=model,
            observations=observations,
            actions=actions,
            epochs=args.epochs_per_iteration,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            writer=writer,
            global_epoch_offset=global_epoch,
        )
        global_epoch += args.epochs_per_iteration
        model.save(args.model_dir / f"prior_dagger_iter_{iteration:02d}")
        current_eval = evaluate_controller(
            model,
            difficulty=args.difficulty,
            episodes=args.eval_episodes,
            max_episode_steps=args.max_episode_steps,
            randomization=args.randomization,
        )
        print_eval(f"prior eval {iteration}", current_eval)
        writer.add_scalar("prior/eval_success_rate", current_eval.success_rate, iteration)
        writer.add_scalar("prior/eval_fit_error", current_eval.average_fit_error, iteration)
        if args.save_best and eval_is_better(current_eval, best_eval):
            best_eval = current_eval
            model.save(best_path)
            print(f"Saved best prior so far to {best_path.with_suffix('.zip')}")

    final_eval = evaluate_controller(
        model,
        difficulty=args.difficulty,
        episodes=args.eval_episodes,
        max_episode_steps=args.max_episode_steps,
        randomization=args.randomization,
    )
    print_eval("final prior gate", final_eval)
    output_path = args.model_dir / args.output_name
    model.save(output_path)
    model.save(args.model_dir / "box_push_bc_pretrained")
    writer.close()
    model.env.close()
    print(f"Saved prior to {output_path.with_suffix('.zip')}")
    if args.save_best and best_eval is not None:
        print_eval("best prior gate", best_eval)
        print(f"Best evaluated prior saved to {best_path.with_suffix('.zip')}")
    print(f"Updated PPO preload model at {args.model_dir / 'box_push_bc_pretrained.zip'}")
    if final_eval.success_rate < args.min_success_rate:
        raise SystemExit(
            f"Prior gate failed: final success {final_eval.success_rate:.1%} is below "
            f"{args.min_success_rate:.1%}. Increase --dagger-iterations or fix the expert."
        )


if __name__ == "__main__":
    main()
