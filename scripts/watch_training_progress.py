import argparse
import time
from pathlib import Path

from stable_baselines3 import PPO

from robot_lab_rl.envs.box_push_env import DIFFICULTIES
from robot_lab_rl.rl import (
    DEFAULT_MODEL_DIR,
    DEFAULT_RESIDUAL_SCALE,
    FlatActionWrapper,
    make_box_push_env,
)
try:
    from manual_drive import ManualDriveApp, UPDATE_MS
except ModuleNotFoundError:
    from scripts.manual_drive import ManualDriveApp, UPDATE_MS


def newest_model_path(model_dir: Path) -> Path | None:
    candidates = [
        *model_dir.glob("box_push_ppo_*_steps.zip"),
        *model_dir.glob("box_push_ppo_after_*.zip"),
        model_dir / "box_push_ppo_best_full.zip",
        model_dir / "box_push_ppo_final.zip",
    ]
    existing = [path for path in candidates if path.exists()]
    if not existing:
        return None
    return max(existing, key=lambda path: path.stat().st_mtime)


class TrainingProgressApp(ManualDriveApp):
    def __init__(
        self,
        env: FlatActionWrapper,
        model_dir: Path,
        model_path: Path | None,
        difficulty: str,
        deterministic: bool,
        reload_seconds: float,
        max_episode_steps: int,
    ) -> None:
        self.model_dir = model_dir
        self.fixed_model_path = model_path
        self.difficulty = difficulty
        self.deterministic = deterministic
        self.reload_seconds = reload_seconds
        self.max_episode_steps = max_episode_steps
        self.model: PPO | None = None
        self.model_path: Path | None = None
        self.last_reload_attempt = 0.0
        self.episode_step = 0
        self.episode_return = 0.0
        self.latest_info: dict = {}
        self.observation, _ = env.reset()
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

        self._reload_model_if_needed()
        if self.model is None:
            self._draw_zone()
            self._draw_objects()
            self._draw_robots()
            self._draw_coordinate_key()
            self._draw_status()
            self.root.after(UPDATE_MS, self._tick)
            return

        action, _ = self.model.predict(self.observation, deterministic=self.deterministic)
        self.observation, reward, terminated, truncated, info = self.wrapped_env.step(action)
        self.latest_info = info
        self.episode_step += 1
        self.episode_return += float(reward)
        self._draw_zone()
        self._draw_objects()
        self._draw_robots()
        self._draw_coordinate_key()
        self._draw_status()
        if terminated or truncated:
            time.sleep(0.35)
            self.observation, _ = self.wrapped_env.reset()
            self.episode_step = 0
            self.episode_return = 0.0
            self.latest_info = {}
            self._reload_model_if_needed(force=True)
        self.root.after(UPDATE_MS, self._tick)

    def _reload_model_if_needed(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_reload_attempt < self.reload_seconds:
            return
        self.last_reload_attempt = now

        candidate = self.fixed_model_path or newest_model_path(self.model_dir)
        if candidate is None or candidate == self.model_path:
            return

        try:
            load_path = candidate.with_suffix("") if candidate.suffix == ".zip" else candidate
            self.model = PPO.load(load_path)
        except (OSError, EOFError, ValueError, RuntimeError) as exc:
            print(f"Could not load {candidate}: {exc}")
            return

        self.model_path = candidate
        print(f"Watching checkpoint: {candidate}")

    def _draw_status(self) -> None:
        model_name = self.model_path.name if self.model_path is not None else "waiting for checkpoint"
        self.canvas.itemconfigure(
            self.status_item,
            text=(
                f"{self.difficulty} | {model_name}\n"
                f"step={self.episode_step:03d} return={self.episode_return:+.2f} "
                f"dist={self.latest_info.get('box_zone_distance', 0.0):.3f} "
                f"fit={self.latest_info.get('box_zone_fit_error', 0.0):.3f} "
                f"success={self.latest_info.get('box_in_zone', False)}"
            ),
        )

    def _on_close(self) -> None:
        self.running = False
        self.wrapped_env.close()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Watch the newest box-push checkpoint while training runs.")
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    parser.add_argument("--model", type=Path, default=None, help="Watch one exact .zip model instead of auto-selecting.")
    parser.add_argument("--difficulty", choices=DIFFICULTIES, default="full")
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument(
        "--randomization",
        type=float,
        default=0.0,
        help="Position jitter in meters for robots/box/zone on each reset.",
    )
    parser.add_argument("--reload-seconds", type=float, default=5.0)
    parser.add_argument("--stochastic", action="store_true", help="Use stochastic actions instead of deterministic ones.")
    parser.add_argument(
        "--residual",
        action="store_true",
        help="Watch residual checkpoints (scripted expert stays in the loop, policy only corrects).",
    )
    parser.add_argument("--residual-scale", type=float, default=DEFAULT_RESIDUAL_SCALE)
    args = parser.parse_args()

    env = make_box_push_env(
        difficulty=args.difficulty,
        max_steps=args.max_episode_steps,
        randomization=args.randomization,
        residual=args.residual,
        residual_scale=args.residual_scale,
    )
    app = TrainingProgressApp(
        env,
        model_dir=args.model_dir,
        model_path=args.model,
        difficulty=args.difficulty,
        deterministic=not args.stochastic,
        reload_seconds=args.reload_seconds,
        max_episode_steps=args.max_episode_steps,
    )
    app.run()


if __name__ == "__main__":
    main()
