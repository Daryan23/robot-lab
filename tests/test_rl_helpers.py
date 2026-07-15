import numpy as np
from stable_baselines3.common.env_checker import check_env

from robot_lab_rl.rl import make_box_push_env


def test_rl_wrapper_flattens_actions_for_stable_baselines():
    env = make_box_push_env(num_robots=3)
    try:
        observation, _ = env.reset()

        assert observation.shape == (21,)
        assert env.action_space.shape == (6,)
        assert np.all(env.action_space.low == -1.0)
        assert np.all(env.action_space.high == 1.0)

        next_observation, reward, terminated, truncated, info = env.step(
            np.zeros(6, dtype=np.float32)
        )

        assert next_observation.shape == (21,)
        assert isinstance(reward, float)
        assert terminated is False
        assert truncated is False
        assert "box_zone_distance" in info
    finally:
        env.close()


def test_rl_wrapper_passes_stable_baselines_env_check():
    env = make_box_push_env()
    try:
        check_env(env, warn=True)
    finally:
        env.close()
