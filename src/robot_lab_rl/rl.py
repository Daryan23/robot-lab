from __future__ import annotations

from pathlib import Path
import re

import numpy as np
from gymnasium import ActionWrapper, spaces

from robot_lab_rl import BoxPushEnv


DEFAULT_MODEL_DIR = Path("models/box_push_ppo")
DEFAULT_LOG_DIR = Path("runs/box_push_ppo")


class FlatActionWrapper(ActionWrapper):
    """Expose normalized flat robot actions for SB3."""

    def __init__(self, env: BoxPushEnv) -> None:
        super().__init__(env)
        self.max_wheel_speed = env.drive.max_wheel_speed
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)

    def action(self, action):
        normalized_action = np.asarray(action, dtype=np.float32).reshape(self.env.action_space.shape)
        return normalized_action * self.max_wheel_speed


def make_box_push_env(
    difficulty: str = "full",
    max_steps: int = 800,
    randomization: float = 0.0,
) -> FlatActionWrapper:
    return FlatActionWrapper(
        BoxPushEnv(
            render_mode="direct",
            difficulty=difficulty,
            max_steps=max_steps,
            randomization=randomization,
        )
    )


def latest_checkpoint(model_dir: Path = DEFAULT_MODEL_DIR) -> Path:
    best_model = model_dir / "box_push_ppo_best_full.zip"
    if best_model.exists():
        return best_model
    final_model = model_dir / "box_push_ppo_final.zip"
    if final_model.exists():
        return final_model
    checkpoints = sorted(model_dir.glob("box_push_ppo_*_steps.zip"), key=_checkpoint_step)
    if checkpoints:
        return checkpoints[-1]
    raise FileNotFoundError(f"No trained model found in {model_dir}")


def _checkpoint_step(path: Path) -> int:
    match = re.search(r"_(\d+)_steps\.zip$", path.name)
    return int(match.group(1)) if match else -1
