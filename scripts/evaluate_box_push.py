import argparse
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from robot_lab_rl.envs.box_push_env import DIFFICULTIES
from robot_lab_rl.rl import DEFAULT_RESIDUAL_SCALE, latest_checkpoint, make_box_push_env


COOP_DIFFICULTIES = ("coop_heavy", "coop_rotate", "coop_mixed")
COOP_SUITE_DIFFICULTIES = (*COOP_DIFFICULTIES, "coop_random")


def evaluate_one(
    model: PPO,
    difficulty: str,
    episodes: int,
    max_episode_steps: int,
    randomization: float,
    residual: bool = False,
    residual_scale: float = DEFAULT_RESIDUAL_SCALE,
) -> dict[str, float]:
    env = make_box_push_env(
        difficulty=difficulty,
        max_steps=max_episode_steps,
        randomization=randomization,
        residual=residual,
        residual_scale=residual_scale,
    )
    successes = 0
    returns: list[float] = []
    lengths: list[int] = []
    final_distances: list[float] = []
    final_fit_errors: list[float] = []
    successful_lengths: list[int] = []

    try:
        for _ in range(episodes):
            observation, _ = env.reset()
            episode_return = 0.0
            info = {}
            for step in range(env.unwrapped.max_steps):
                action, _ = model.predict(observation, deterministic=True)
                observation, reward, terminated, truncated, info = env.step(action)
                episode_return += float(reward)
                if terminated or truncated:
                    break
            success = bool(info.get("box_in_zone", False))
            successes += int(success)
            returns.append(episode_return)
            lengths.append(step + 1)
            if success:
                successful_lengths.append(step + 1)
            final_distances.append(float(info.get("box_zone_distance", np.nan)))
            final_fit_errors.append(float(info.get("box_zone_fit_error", np.nan)))
    finally:
        env.close()

    return {
        "success_rate": successes / episodes,
        "average_return": float(np.mean(returns)),
        "average_length": float(np.mean(lengths)),
        "average_success_length": float(np.mean(successful_lengths)) if successful_lengths else float("nan"),
        "average_distance": float(np.nanmean(final_distances)),
        "average_fit_error": float(np.nanmean(final_fit_errors)),
    }


def print_result(difficulty: str, result: dict[str, float]) -> None:
    print(f"Difficulty: {difficulty}")
    print(f"Success rate: {result['success_rate']:.2%}")
    print(f"Average return: {result['average_return']:.3f}")
    print(f"Average length: {result['average_length']:.1f} steps")
    print(f"Average successful length: {result['average_success_length']:.1f} steps")
    print(f"Average final box-zone distance: {result['average_distance']:.3f} m")
    print(f"Average final fit error: {result['average_fit_error']:.3f} m")


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate a trained box-push PPO model.")
    parser.add_argument("--model", type=Path, default=None, help="Path to a .zip model checkpoint.")
    parser.add_argument("--episodes", type=int, default=20, help="Number of evaluation episodes.")
    parser.add_argument("--difficulty", choices=(*DIFFICULTIES, "all", "coop", "coop_suite"), default="full")
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument(
        "--randomization",
        type=float,
        default=0.0,
        help="Position jitter in meters for robots/box/zone on each reset.",
    )
    parser.add_argument(
        "--residual",
        action="store_true",
        help="Evaluate a residual policy (scripted expert stays in the loop, policy only corrects).",
    )
    parser.add_argument("--residual-scale", type=float, default=DEFAULT_RESIDUAL_SCALE)
    args = parser.parse_args()

    model_path = args.model or latest_checkpoint()
    model = PPO.load(model_path)
    print(f"Model: {model_path}")
    print(f"Episodes: {args.episodes}")
    if args.difficulty == "all":
        difficulties = DIFFICULTIES
    elif args.difficulty == "coop":
        difficulties = COOP_DIFFICULTIES
    elif args.difficulty == "coop_suite":
        difficulties = COOP_SUITE_DIFFICULTIES
    else:
        difficulties = (args.difficulty,)

    results = [
        evaluate_one(
            model,
            difficulty,
            args.episodes,
            args.max_episode_steps,
            args.randomization,
            residual=args.residual,
            residual_scale=args.residual_scale,
        )
        for difficulty in difficulties
    ]
    for difficulty, result in zip(difficulties, results):
        print_result(difficulty, result)
    if len(results) > 1:
        print("Combined success rate:", f"{np.mean([result['success_rate'] for result in results]):.2%}")


if __name__ == "__main__":
    main()
