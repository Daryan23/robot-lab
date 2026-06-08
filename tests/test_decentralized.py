import numpy as np
import pytest

from robot_lab_rl.decentralized import (
    EGO_OBS_DIM,
    SharedBoxPushVecEnv,
    egocentric_observation,
    make_shared_box_push_vec_env,
)
from robot_lab_rl.envs.box_push_env import BoxPushEnv


def test_vec_env_exposes_one_subenv_per_robot():
    vec_env = make_shared_box_push_vec_env(difficulty="easy", max_steps=50)
    try:
        assert vec_env.num_envs == 2
        assert vec_env.observation_space.shape == (EGO_OBS_DIM,)
        assert vec_env.action_space.shape == (2,)
        observations = vec_env.reset()
        assert observations.shape == (2, EGO_OBS_DIM)
    finally:
        vec_env.close()


def test_step_returns_per_agent_arrays_and_shared_reward():
    vec_env = make_shared_box_push_vec_env(difficulty="easy", max_steps=50)
    try:
        vec_env.reset()
        actions = np.zeros((2, 2), dtype=np.float32)
        observations, rewards, dones, infos = vec_env.step(actions)
        assert observations.shape == (2, EGO_OBS_DIM)
        assert rewards.shape == (2,)
        assert dones.shape == (2,)
        # Team reward: every agent receives the same scalar.
        assert rewards[0] == pytest.approx(rewards[1])
        assert len(infos) == 2
        assert infos[0]["agent_index"] == 0
        assert infos[1]["agent_index"] == 1
    finally:
        vec_env.close()


def test_egocentric_observation_is_local_not_global():
    env = BoxPushEnv(difficulty="easy", max_steps=50)
    try:
        env.reset()
        obs0 = egocentric_observation(env, 0)
        obs1 = egocentric_observation(env, 1)
        assert obs0.shape == (EGO_OBS_DIM,)
        # Two robots at different places must see different local worlds,
        # otherwise a shared policy could never act differently.
        assert not np.allclose(obs0, obs1)
    finally:
        env.close()


def test_short_rollout_runs_without_error():
    vec_env = SharedBoxPushVecEnv(BoxPushEnv(difficulty="easy", max_steps=10))
    try:
        vec_env.reset()
        for _ in range(15):  # longer than max_steps to cross an auto-reset
            actions = np.full((2, 2), 0.5, dtype=np.float32)
            _, _, dones, infos = vec_env.step(actions)
            if dones.any():
                assert "terminal_observation" in infos[0]
    finally:
        vec_env.close()
