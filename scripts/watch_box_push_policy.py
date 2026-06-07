import argparse
import time
from pathlib import Path

from stable_baselines3 import PPO

from robot_lab_rl import BoxPushEnv
from robot_lab_rl.envs.box_push_env import DIFFICULTIES
from robot_lab_rl.rl import FlatActionWrapper, latest_checkpoint
try:
    from manual_drive import ManualDriveApp, UPDATE_MS
except ModuleNotFoundError:
    from scripts.manual_drive import ManualDriveApp, UPDATE_MS


class PolicyDriveApp(ManualDriveApp):
    def __init__(self, env: FlatActionWrapper, model: PPO, deterministic: bool) -> None:
        self.model = model
        self.deterministic = deterministic
        self.observation, _ = env.reset()
        super().__init__(env.unwrapped)
        self.wrapped_env = env

    def _tick(self) -> None:
        if not self.running:
            return

        action, _ = self.model.predict(self.observation, deterministic=self.deterministic)
        self.observation, _, terminated, truncated, _ = self.wrapped_env.step(action)
        self._draw_zone()
        self._draw_objects()
        self._draw_robots()
        self._draw_coordinate_key()
        if terminated or truncated:
            time.sleep(0.5)
            self.observation, _ = self.wrapped_env.reset()
        self.root.after(UPDATE_MS, self._tick)

    def _on_close(self) -> None:
        self.running = False
        self.wrapped_env.close()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Watch a trained box-push PPO model in the 2D viewer.")
    parser.add_argument("--model", type=Path, default=None, help="Path to a .zip model checkpoint.")
    parser.add_argument("--difficulty", choices=DIFFICULTIES, default="full")
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument("--stochastic", action="store_true", help="Use stochastic actions instead of deterministic ones.")
    args = parser.parse_args()

    model_path = args.model or latest_checkpoint()
    env = FlatActionWrapper(
        BoxPushEnv(render_mode="direct", difficulty=args.difficulty, max_steps=args.max_episode_steps)
    )
    model = PPO.load(model_path)
    print(f"Watching model: {model_path}")
    app = PolicyDriveApp(env, model, deterministic=not args.stochastic)
    app.run()


if __name__ == "__main__":
    main()
