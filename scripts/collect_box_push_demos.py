import argparse
import time
from pathlib import Path

import numpy as np

from robot_lab_rl import BoxPushEnv, BoxPushHeuristicPolicy
from robot_lab_rl.envs.box_push_env import DIFFICULTIES
from robot_lab_rl.rl import FlatActionWrapper
try:
    from manual_drive import ManualDriveApp, UPDATE_MS
except ModuleNotFoundError:
    from scripts.manual_drive import ManualDriveApp, UPDATE_MS


DIFFICULTY_IDS = {difficulty: index for index, difficulty in enumerate(DIFFICULTIES)}


def collect_demos(
    *,
    steps: int,
    difficulties: list[str],
    output_path: Path,
    max_episode_steps: int,
    require_success: bool,
    max_attempts_per_difficulty: int,
    randomization: float,
) -> None:
    expert = BoxPushHeuristicPolicy()
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    difficulty_ids: list[int] = []
    episode_returns: list[float] = []
    successes: list[bool] = []

    base_steps = max(1, steps // len(difficulties))
    remainder = steps - base_steps * len(difficulties)
    for difficulty_index, difficulty in enumerate(difficulties):
        steps_for_difficulty = base_steps + (1 if difficulty_index < remainder else 0)
        env = FlatActionWrapper(
            BoxPushEnv(
                render_mode="direct",
                difficulty=difficulty,
                max_steps=max_episode_steps,
                randomization=randomization,
            )
        )
        try:
            collected = 0
            attempts = 0
            while collected < steps_for_difficulty and attempts < max_attempts_per_difficulty:
                attempts += 1
                observation, _ = env.reset()
                initial_distance = env.unwrapped.box_zone_distance()
                episode_observations: list[np.ndarray] = []
                episode_actions: list[np.ndarray] = []
                episode_return = 0.0
                success = False
                info = {"box_zone_distance": initial_distance}

                for _ in range(max_episode_steps):
                    action = expert.predict(observation)
                    next_observation, reward, terminated, truncated, info = env.step(action)
                    episode_observations.append(observation.copy())
                    episode_actions.append(action.copy())
                    episode_return += float(reward)
                    observation = next_observation
                    if terminated or truncated:
                        success = bool(info.get("box_in_zone", False))
                        break

                useful_episode = success or (
                    not require_success
                    and info.get("box_zone_distance", initial_distance) < initial_distance
                )
                if useful_episode:
                    keep = min(len(episode_observations), steps_for_difficulty - collected)
                    observations.extend(episode_observations[:keep])
                    actions.extend(episode_actions[:keep])
                    difficulty_ids.extend([DIFFICULTY_IDS[difficulty]] * keep)
                    episode_returns.append(episode_return)
                    successes.append(success)
                    collected += keep

                print(
                    f"{difficulty}: collected={collected}/{steps_for_difficulty}, "
                    f"last_return={episode_return:+.2f}, success={success}"
                )
            if collected < steps_for_difficulty:
                print(
                    f"{difficulty}: stopped early after {attempts} attempts; "
                    f"collected {collected}/{steps_for_difficulty} usable steps"
                )
        finally:
            env.close()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        observations=np.asarray(observations, dtype=np.float32),
        actions=np.asarray(actions, dtype=np.float32),
        difficulties=np.asarray(difficulty_ids, dtype=np.int64),
        episode_returns=np.asarray(episode_returns, dtype=np.float32),
        successes=np.asarray(successes, dtype=bool),
    )
    print(f"Saved {len(observations)} demo steps to {output_path}")


class VisualDemoCollectionApp(ManualDriveApp):
    def __init__(
        self,
        env: FlatActionWrapper,
        expert: BoxPushHeuristicPolicy,
        steps: int,
        difficulty: str,
        output_path: Path,
    ) -> None:
        self.expert = expert
        self.target_steps = steps
        self.difficulty = difficulty
        self.output_path = output_path
        self.collected_steps = 0
        self.observation, _ = env.reset()
        self.observations: list[np.ndarray] = []
        self.actions: list[np.ndarray] = []
        super().__init__(env.unwrapped)
        self.wrapped_env = env
        self.status_item = self.canvas.create_text(12, 12, anchor="nw", fill="#111111", font=("Menlo", 12, "bold"))

    def _tick(self) -> None:
        if not self.running:
            return
        action = self.expert.predict(self.observation)
        self.observations.append(self.observation.copy())
        self.actions.append(action.copy())
        self.observation, _, terminated, truncated, info = self.wrapped_env.step(action)
        self.collected_steps += 1
        self._draw_zone()
        self._draw_objects()
        self._draw_robots()
        self._draw_coordinate_key()
        self.canvas.itemconfigure(
            self.status_item,
            text=(
                f"Visual demo collection | steps={self.collected_steps}/{self.target_steps} | "
                f"dist={info.get('box_zone_distance', 0.0):.3f}"
            ),
        )
        if terminated or truncated:
            time.sleep(0.35)
            self.observation, _ = self.wrapped_env.reset()
        if self.collected_steps >= self.target_steps:
            self._save()
            self._on_close()
            return
        self.root.after(UPDATE_MS, self._tick)

    def _save(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            self.output_path,
            observations=np.asarray(self.observations, dtype=np.float32),
            actions=np.asarray(self.actions, dtype=np.float32),
            difficulties=np.full(len(self.observations), DIFFICULTY_IDS[self.difficulty], dtype=np.int64),
            episode_returns=np.asarray([], dtype=np.float32),
            successes=np.asarray([], dtype=bool),
        )
        print(f"Saved {len(self.observations)} visual demo steps to {self.output_path}")

    def _on_close(self) -> None:
        self.running = False
        self.wrapped_env.close()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect scripted expert demonstrations for box-push BC.")
    parser.add_argument("--steps", type=int, default=200_000)
    parser.add_argument("--output", type=Path, default=Path("data/box_push_demos.npz"))
    parser.add_argument("--difficulty", choices=(*DIFFICULTIES, "all", "coop"), default="easy")
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument(
        "--include-progress-episodes",
        action="store_true",
        help="Also keep non-success episodes that move the box closer.",
    )
    parser.add_argument("--max-attempts-per-difficulty", type=int, default=2_000)
    parser.add_argument(
        "--randomization",
        type=float,
        default=0.015,
        help="Position jitter in meters for robots/box/zone during data collection.",
    )
    parser.add_argument("--visual", action="store_true", help="Watch expert collection for one difficulty.")
    args = parser.parse_args()

    if args.difficulty == "all":
        difficulties = list(DIFFICULTIES)
    elif args.difficulty == "coop":
        difficulties = ["coop_heavy", "coop_rotate", "coop_mixed"]
    else:
        difficulties = [args.difficulty]
    if args.visual:
        difficulty = difficulties[0]
        env = FlatActionWrapper(
            BoxPushEnv(
                render_mode="direct",
                difficulty=difficulty,
                max_steps=args.max_episode_steps,
                randomization=args.randomization,
            )
        )
        VisualDemoCollectionApp(env, BoxPushHeuristicPolicy(), args.steps, difficulty, args.output).run()
        return

    collect_demos(
        steps=args.steps,
        difficulties=difficulties,
        output_path=args.output,
        max_episode_steps=args.max_episode_steps,
        require_success=not args.include_progress_episodes,
        max_attempts_per_difficulty=args.max_attempts_per_difficulty,
        randomization=args.randomization,
    )


if __name__ == "__main__":
    main()
