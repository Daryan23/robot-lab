from __future__ import annotations

from pathlib import Path
import re

import gymnasium as gym
import numpy as np
from gymnasium import ActionWrapper, spaces

from robot_lab_rl import BoxPushEnv, BoxPushHeuristicPolicy
from robot_lab_rl.envs.box_push_env import DEFAULT_NUM_ROBOTS


DEFAULT_MODEL_DIR = Path("models/box_push_ppo")
DEFAULT_LOG_DIR = Path("runs/box_push_ppo")
DEFAULT_RESIDUAL_SCALE = 0.3


class FlatActionWrapper(ActionWrapper):
    """Expose normalized flat robot actions for SB3."""

    def __init__(self, env: BoxPushEnv) -> None:
        super().__init__(env)
        self.max_wheel_speed = env.drive.max_wheel_speed
        self.flat_action_dim = 2 * env.num_robots
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.flat_action_dim,), dtype=np.float32
        )

    def action(self, action):
        normalized_action = np.asarray(action, dtype=np.float32).reshape(self.env.action_space.shape)
        return normalized_action * self.max_wheel_speed


class ResidualActionWrapper(gym.Wrapper):
    """Keep the scripted expert in the control loop; the policy only corrects it.

    The executed (normalized) action is::

        action = clip(expert(obs) + residual_scale * policy_residual, -1, 1)

    A zero policy output therefore reproduces the hard-coded expert exactly, so
    the network only has to learn *corrections*. ``residual_scale`` bounds how
    far the policy may pull the action away from the expert, which keeps the
    hand-written controller dominant.

    The per-step ``info`` dict is augmented with the expert action, the raw
    residual, the applied correction and its magnitude/saturation so training
    and evaluation can visualise what the network is changing.
    """

    def __init__(
        self,
        env: FlatActionWrapper,
        expert: BoxPushHeuristicPolicy | None = None,
        residual_scale: float = DEFAULT_RESIDUAL_SCALE,
    ) -> None:
        super().__init__(env)
        if residual_scale <= 0.0:
            raise ValueError("residual_scale must be positive.")
        self.expert = expert or BoxPushHeuristicPolicy()
        self.residual_scale = float(residual_scale)
        self.action_dim = 2 * env.unwrapped.num_robots
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.action_dim,), dtype=np.float32
        )
        self._last_observation: np.ndarray | None = None

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        observation, info = self.env.reset(seed=seed, options=options)
        self._last_observation = np.asarray(observation, dtype=np.float32)
        return observation, info

    def step(self, residual):
        if self._last_observation is None:
            raise RuntimeError("Call reset() before step().")
        residual = np.asarray(residual, dtype=np.float32).reshape(self.action_dim)
        expert_action = np.asarray(
            self.expert.predict(self._last_observation), dtype=np.float32
        ).reshape(self.action_dim)
        correction = self.residual_scale * residual
        composed = np.clip(expert_action + correction, -1.0, 1.0).astype(np.float32)
        observation, reward, terminated, truncated, info = self.env.step(composed)
        self._last_observation = np.asarray(observation, dtype=np.float32)
        info = {
            **info,
            "expert_action": expert_action,
            "residual_action": residual,
            "applied_correction": correction,
            "composed_action": composed,
            "correction_magnitude": float(np.mean(np.abs(correction))),
            "correction_saturation": float(np.mean(np.abs(composed) >= 0.999)),
        }
        return observation, reward, terminated, truncated, info


def make_box_push_env(
    difficulty: str = "full",
    max_steps: int = 800,
    randomization: float = 0.0,
    residual: bool = False,
    residual_scale: float = DEFAULT_RESIDUAL_SCALE,
    expert: BoxPushHeuristicPolicy | None = None,
    num_robots: int = DEFAULT_NUM_ROBOTS,
):
    env = FlatActionWrapper(
        BoxPushEnv(
            render_mode="direct",
            difficulty=difficulty,
            max_steps=max_steps,
            randomization=randomization,
            num_robots=num_robots,
        )
    )
    if residual:
        return ResidualActionWrapper(env, expert=expert, residual_scale=residual_scale)
    return env


def zero_init_residual_policy(model, log_std_init: float | None = None) -> None:
    """Make a fresh policy start as a no-op correction (action ≈ expert).

    Zeroing the action mean head means the initial residual is ~0, so the
    residual policy begins at expert performance instead of from a random
    policy. Optionally lower the action log-std for gentler initial exploration.
    """
    import torch

    with torch.no_grad():
        model.policy.action_net.weight.zero_()
        model.policy.action_net.bias.zero_()
        if log_std_init is not None:
            model.policy.log_std.fill_(float(log_std_init))


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
