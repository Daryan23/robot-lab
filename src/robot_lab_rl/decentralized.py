"""Decentralized, parameter-sharing view on the box-push task.

The base :class:`BoxPushEnv` exposes a single global observation and a single
flat action vector for *both* robots, which makes it a centralized controller.
This module turns the very same physical world into a *parameter-sharing*
multi-agent setup:

* one shared policy network,
* called once per robot with that robot's own **egocentric** observation,
* trained on the pooled experience of all robots (a shared team reward).

Concretely we present the ``n`` robots to Stable-Baselines3 as ``n`` parallel
sub-environments of a single :class:`VecEnv`. SB3 then learns one network from
all of them at once -- exactly the parameter-sharing idea. Because the network
only ever sees a single robot's local view, execution stays decentralized and
the same network scales to ``n > 2`` robots without any change.
"""

from __future__ import annotations

import math

import numpy as np
from gymnasium import spaces
from stable_baselines3.common.vec_env.base_vec_env import (
    VecEnv,
    VecEnvObs,
    VecEnvStepReturn,
)

from robot_lab_rl.envs.box_push_env import DEFAULT_NUM_ROBOTS, BoxPushEnv

# How many nearest other robots each robot sees in its egocentric observation.
DEFAULT_NUM_NEIGHBORS = 2

# One egocentric observation = 11 self/box/zone features, then 3 numbers per
# included neighbor (relative x, y, heading), then 3 box-geometry numbers.
_EGO_BASE_DIM = 11
_EGO_GEOMETRY_DIM = 3


def ego_obs_dim(num_neighbors: int = DEFAULT_NUM_NEIGHBORS) -> int:
    """Length of one robot's egocentric observation for ``num_neighbors``."""
    return _EGO_BASE_DIM + 3 * num_neighbors + _EGO_GEOMETRY_DIM


# Default egocentric observation length (two nearest neighbors).
EGO_OBS_DIM = ego_obs_dim()


def _to_robot_frame(dx: float, dy: float, yaw: float) -> tuple[float, float]:
    """Rotate a world-frame vector into the robot's heading frame.

    After rotation, ``+x`` always points straight ahead of the robot, so the
    same policy rule ("push what is in front of me") works regardless of how
    the robot is currently oriented.
    """
    cos_a = math.cos(yaw)
    sin_a = math.sin(yaw)
    return dx * cos_a + dy * sin_a, -dx * sin_a + dy * cos_a


def _normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def egocentric_observation(
    env: BoxPushEnv,
    robot_index: int,
    num_neighbors: int = DEFAULT_NUM_NEIGHBORS,
) -> np.ndarray:
    """Build the local observation of one robot, from *its* point of view.

    Everything is expressed relative to the robot and rotated into its heading
    frame, so the robot spans its *own* coordinate system and never sees
    absolute world coordinates (the kind of signal a real e-puck-style sensor
    would produce). We include the ``num_neighbors`` *nearest* other robots,
    sorted by distance, which keeps the dimensionality fixed for any team size.
    If fewer other robots exist, the remaining neighbor slots are zero-padded.
    """
    robot_x, robot_y, yaw = env.robot_states[robot_index]
    box = env.box
    zone_x, zone_y = env.zone_position

    other_indices = sorted(
        (j for j in range(env.num_robots) if j != robot_index),
        key=lambda j: math.hypot(
            env.robot_states[j][0] - robot_x,
            env.robot_states[j][1] - robot_y,
        ),
    )

    box_rel_x, box_rel_y = _to_robot_frame(box.x - robot_x, box.y - robot_y, yaw)
    box_vel_x, box_vel_y = _to_robot_frame(box.vx, box.vy, yaw)
    zone_rel_x, zone_rel_y = _to_robot_frame(zone_x - robot_x, zone_y - robot_y, yaw)

    features = [
        math.cos(yaw),
        math.sin(yaw),
        box_rel_x,
        box_rel_y,
        _normalize_angle(box.angle - yaw),
        box_vel_x,
        box_vel_y,
        box.omega,
        zone_rel_x,
        zone_rel_y,
        _normalize_angle(env.target_angle - yaw),
    ]

    for slot in range(num_neighbors):
        if slot < len(other_indices):
            other_x, other_y, other_yaw = env.robot_states[other_indices[slot]]
            other_rel_x, other_rel_y = _to_robot_frame(other_x - robot_x, other_y - robot_y, yaw)
            features.extend([other_rel_x, other_rel_y, _normalize_angle(other_yaw - yaw)])
        else:
            features.extend([0.0, 0.0, 0.0])

    features.extend([box.half_width, box.half_height, box.mass])
    return np.array(features, dtype=np.float32)


class SharedBoxPushVecEnv(VecEnv):
    """Expose the ``n`` robots of one :class:`BoxPushEnv` as ``n`` sub-envs.

    Each "sub-environment" is one robot's egocentric view of the shared world.
    A single :class:`BoxPushEnv` is stepped once per ``step``; its scalar team
    reward and its done flag are broadcast to every agent. SB3 trains one
    shared policy from the pooled transitions of all robots.
    """

    def __init__(
        self,
        env: BoxPushEnv,
        residual_scale: float = 0.0,
        num_neighbors: int = DEFAULT_NUM_NEIGHBORS,
    ) -> None:
        self._env = env
        self._num_robots = env.num_robots
        self._num_neighbors = num_neighbors
        self._max_wheel_speed = env.drive.max_wheel_speed
        # Residual Policy Learning: if > 0, the policy only adds a correction on
        # top of the geometric expert, so it starts at expert performance.
        self._residual_scale = float(residual_scale)
        self._expert = None
        if self._residual_scale > 0.0:
            from robot_lab_rl import BoxPushHeuristicPolicy

            self._expert = BoxPushHeuristicPolicy()
        observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(ego_obs_dim(num_neighbors),),
            dtype=np.float32,
        )
        action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        super().__init__(self._num_robots, observation_space, action_space)
        self._pending_actions: np.ndarray | None = None

    # -- observation assembly ------------------------------------------------
    def _all_observations(self) -> np.ndarray:
        return np.stack(
            [
                egocentric_observation(self._env, i, self._num_neighbors)
                for i in range(self._num_robots)
            ]
        ).astype(np.float32)

    def actions_to_wheel_speeds(self, env: BoxPushEnv, policy_actions) -> np.ndarray:
        """Map normalized policy actions to raw wheel speeds.

        In residual mode the policy output is added as a correction on top of
        the geometric expert's command; otherwise it *is* the command.
        """
        policy = np.clip(
            np.asarray(policy_actions, dtype=np.float32), -1.0, 1.0
        ).reshape(env.num_robots, 2)
        if self._expert is not None:
            expert_norm = np.asarray(
                self._expert.predict(env._get_observation()), dtype=np.float32
            ).reshape(env.num_robots, 2)
            combined = np.clip(expert_norm + self._residual_scale * policy, -1.0, 1.0)
        else:
            combined = policy
        return combined * self._max_wheel_speed

    # -- VecEnv API ----------------------------------------------------------
    def reset(self) -> VecEnvObs:
        self._env.reset()
        return self._all_observations()

    def step_async(self, actions: np.ndarray) -> None:
        self._pending_actions = np.asarray(actions, dtype=np.float32)

    def step_wait(self) -> VecEnvStepReturn:
        assert self._pending_actions is not None, "Call step_async() before step_wait()."
        wheel_speeds = self.actions_to_wheel_speeds(self._env, self._pending_actions)
        _, team_reward, terminated, truncated, info = self._env.step(wheel_speeds)
        done = bool(terminated or truncated)

        observations = self._all_observations()
        rewards = np.full(self._num_robots, float(team_reward), dtype=np.float32)
        dones = np.full(self._num_robots, done, dtype=bool)
        infos: list[dict] = []
        for i in range(self._num_robots):
            agent_info = dict(info)
            agent_info["agent_index"] = i
            infos.append(agent_info)

        if done:
            # SB3 VecEnv convention: auto-reset and stash the terminal obs.
            terminal_observations = observations
            self._env.reset()
            observations = self._all_observations()
            for i in range(self._num_robots):
                infos[i]["terminal_observation"] = terminal_observations[i]
                infos[i]["TimeLimit.truncated"] = truncated and not terminated

        return observations, rewards, dones, infos

    def close(self) -> None:
        self._env.close()

    def _resolve_indices(self, indices) -> list[int]:
        if indices is None:
            return list(range(self._num_robots))
        if isinstance(indices, int):
            return [indices]
        return list(indices)

    def get_attr(self, attr_name: str, indices=None) -> list:
        value = getattr(self._env, attr_name)
        return [value for _ in self._resolve_indices(indices)]

    def set_attr(self, attr_name: str, value, indices=None) -> None:
        setattr(self._env, attr_name, value)

    def env_method(self, method_name: str, *method_args, indices=None, **method_kwargs) -> list:
        method = getattr(self._env, method_name)
        result = method(*method_args, **method_kwargs)
        return [result for _ in self._resolve_indices(indices)]

    def env_is_wrapped(self, wrapper_class, indices=None) -> list[bool]:
        return [False for _ in self._resolve_indices(indices)]


def make_shared_box_push_vec_env(
    difficulty: str = "full",
    max_steps: int = 800,
    randomization: float = 0.0,
    num_robots: int = DEFAULT_NUM_ROBOTS,
    residual_scale: float = 0.0,
    num_neighbors: int = DEFAULT_NUM_NEIGHBORS,
) -> SharedBoxPushVecEnv:
    """Build a parameter-sharing VecEnv around a single box-push world."""
    env = BoxPushEnv(
        render_mode="direct",
        difficulty=difficulty,
        max_steps=max_steps,
        randomization=randomization,
        num_robots=num_robots,
    )
    return SharedBoxPushVecEnv(
        env, residual_scale=residual_scale, num_neighbors=num_neighbors
    )
