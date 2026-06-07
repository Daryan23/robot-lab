import argparse
import time

from robot_lab_rl import BoxPushEnv, BoxPushHeuristicPolicy
from robot_lab_rl.envs.box_push_env import DIFFICULTIES
from robot_lab_rl.rl import FlatActionWrapper
try:
    from manual_drive import ManualDriveApp, UPDATE_MS
except ModuleNotFoundError:
    from scripts.manual_drive import ManualDriveApp, UPDATE_MS


class ExpertDriveApp(ManualDriveApp):
    def __init__(self, env: FlatActionWrapper, expert: BoxPushHeuristicPolicy) -> None:
        self.expert = expert
        self.observation, _ = env.reset()
        self.episode_return = 0.0
        self.episode_step = 0
        super().__init__(env.unwrapped)
        self.wrapped_env = env
        self.status_item = self.canvas.create_text(
            12,
            12,
            anchor="nw",
            fill="#111111",
            font=("Menlo", 12, "bold"),
            text="",
        )

    def _tick(self) -> None:
        if not self.running:
            return

        action = self.expert.predict(self.observation)
        self.observation, reward, terminated, truncated, info = self.wrapped_env.step(action)
        self.episode_return += float(reward)
        self.episode_step += 1
        self._draw_zone()
        self._draw_objects()
        self._draw_robots()
        self._draw_coordinate_key()
        self.canvas.itemconfigure(
            self.status_item,
            text=(
                f"Expert | step={self.episode_step:03d} | "
                f"return={self.episode_return:+.2f} | "
                f"dist={info.get('box_zone_distance', 0.0):.3f} | "
                f"fit={info.get('box_zone_fit_error', 0.0):.3f} | "
                f"success={info.get('box_in_zone', False)}"
            ),
        )
        if terminated or truncated:
            time.sleep(0.45)
            self.observation, _ = self.wrapped_env.reset()
            self.episode_return = 0.0
            self.episode_step = 0
        self.root.after(UPDATE_MS, self._tick)

    def _on_close(self) -> None:
        self.running = False
        self.wrapped_env.close()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Watch the scripted box-push expert.")
    parser.add_argument("--difficulty", choices=DIFFICULTIES, default="easy")
    args = parser.parse_args()

    env = FlatActionWrapper(BoxPushEnv(render_mode="direct", difficulty=args.difficulty, max_steps=800))
    app = ExpertDriveApp(env, BoxPushHeuristicPolicy())
    app.run()


if __name__ == "__main__":
    main()
