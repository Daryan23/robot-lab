"""Watch a decentralized, parameter-sharing box-push policy in the 2D viewer.

The shared network is called once per robot with that robot's egocentric
observation -- exactly as during training and as it would run on real,
independently-sensing robots.

Example:
    python scripts/watch_shared_policy.py --model models/box_push_shared_ppo/box_push_shared_ppo_full_final.zip
"""

import argparse
import time
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from robot_lab_rl import BoxPushEnv
from robot_lab_rl.decentralized import SharedBoxPushVecEnv, egocentric_observation
from robot_lab_rl.envs.box_push_env import DIFFICULTIES

try:
    from manual_drive import ManualDriveApp, UPDATE_MS
except ModuleNotFoundError:
    from scripts.manual_drive import ManualDriveApp, UPDATE_MS


class SharedPolicyDriveApp(ManualDriveApp):
    """Drive both robots from one shared policy, each with its own local view."""

    def __init__(self, env: BoxPushEnv, model: PPO, deterministic: bool, residual_scale: float) -> None:
        self.model = model
        self.deterministic = deterministic
        # Reuse the training-time action blending (handles residual mode).
        self._blender = SharedBoxPushVecEnv(env, residual_scale=residual_scale)
        env.reset()
        super().__init__(env)

    def _tick(self) -> None:
        if not self.running:
            return

        # One forward pass per robot, each on its own egocentric observation.
        observations = np.stack(
            [egocentric_observation(self.env, i) for i in range(self.env.num_robots)]
        )
        actions, _ = self.model.predict(observations, deterministic=self.deterministic)
        wheel_speeds = self._blender.actions_to_wheel_speeds(self.env, actions)

        _, _, terminated, truncated, _ = self.env.step(wheel_speeds)
        self._draw_zone()
        self._draw_objects()
        self._draw_robots()
        self._draw_coordinate_key()
        if terminated or truncated:
            time.sleep(0.5)
            self.env.reset()
        self.root.after(UPDATE_MS, self._tick)

    def _on_close(self) -> None:
        self.running = False
        self.env.close()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True, help="Path to a .zip shared-policy checkpoint.")
    parser.add_argument("--difficulty", choices=DIFFICULTIES, default="full")
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument("--randomization", type=float, default=0.0)
    parser.add_argument("--stochastic", action="store_true", help="Sample actions instead of using the mean.")
    parser.add_argument(
        "--residual-scale",
        type=float,
        default=0.0,
        help="Must match the value used during training for a residual policy.",
    )
    args = parser.parse_args()

    env = BoxPushEnv(
        render_mode="direct",
        difficulty=args.difficulty,
        max_steps=args.max_episode_steps,
        randomization=args.randomization,
    )
    model = PPO.load(args.model)
    print(f"Watching shared policy: {args.model}")
    app = SharedPolicyDriveApp(
        env, model, deterministic=not args.stochastic, residual_scale=args.residual_scale
    )
    app.run()


if __name__ == "__main__":
    main()
