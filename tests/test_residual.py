import numpy as np
import pytest

from robot_lab_rl import BoxPushHeuristicPolicy
from robot_lab_rl.rl import ResidualActionWrapper, make_box_push_env, zero_init_residual_policy


def test_make_residual_env_action_space():
    env = make_box_push_env(difficulty="easy", max_steps=50, residual=True, residual_scale=0.3)
    try:
        assert isinstance(env, ResidualActionWrapper)
        assert env.action_space.shape == (6,)
        assert np.allclose(env.action_space.low, -1.0)
        assert np.allclose(env.action_space.high, 1.0)
        # The observation space is unchanged by the residual wrapper.
        assert env.observation_space.shape == (21,)
    finally:
        env.close()


def test_zero_residual_reproduces_expert():
    env = make_box_push_env(difficulty="easy", max_steps=50, residual=True, residual_scale=0.5)
    try:
        observation, _ = env.reset(seed=0)
        expert = BoxPushHeuristicPolicy()
        expected = np.asarray(expert.predict(observation), dtype=np.float32).reshape(6)
        _, _, _, _, info = env.step(np.zeros(6, dtype=np.float32))
        assert np.allclose(info["composed_action"], np.clip(expected, -1.0, 1.0), atol=1e-5)
        assert np.allclose(info["expert_action"], expected, atol=1e-5)
        assert info["correction_magnitude"] == pytest.approx(0.0)
    finally:
        env.close()


def test_residual_correction_is_scaled_and_added():
    scale = 0.4
    env = make_box_push_env(difficulty="easy", max_steps=50, residual=True, residual_scale=scale)
    try:
        observation, _ = env.reset(seed=1)
        expert = BoxPushHeuristicPolicy()
        expert_action = np.asarray(expert.predict(observation), dtype=np.float32).reshape(6)
        residual = np.ones(6, dtype=np.float32)
        _, _, _, _, info = env.step(residual)
        expected = np.clip(expert_action + scale * residual, -1.0, 1.0)
        assert np.allclose(info["composed_action"], expected, atol=1e-5)
        assert np.allclose(info["applied_correction"], scale * residual, atol=1e-5)
        assert info["correction_magnitude"] == pytest.approx(scale)
    finally:
        env.close()


def test_step_before_reset_raises():
    env = make_box_push_env(difficulty="easy", max_steps=50, residual=True)
    try:
        with pytest.raises(RuntimeError):
            env.step(np.zeros(6, dtype=np.float32))
    finally:
        env.close()


def test_invalid_residual_scale_raises():
    with pytest.raises(ValueError):
        make_box_push_env(difficulty="easy", residual=True, residual_scale=0.0)


def test_zero_init_policy_starts_at_expert():
    from stable_baselines3 import PPO

    env = make_box_push_env(difficulty="easy", max_steps=50, residual=True, residual_scale=0.3)
    try:
        model = PPO(
            "MlpPolicy",
            env,
            n_steps=64,
            batch_size=32,
            policy_kwargs={"net_arch": [64, 64]},
        )
        zero_init_residual_policy(model, log_std_init=-1.0)
        observation, _ = env.reset(seed=2)
        action, _ = model.predict(observation, deterministic=True)
        # A zeroed correction head emits a ~0 residual, i.e. action ≈ expert.
        assert np.allclose(action, 0.0, atol=1e-6)
    finally:
        env.close()
