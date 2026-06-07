import numpy as np
import pytest
from stable_baselines3 import PPO

from robot_lab_rl import BoxPushEnv, BoxPushHeuristicPolicy
from robot_lab_rl.rl import FlatActionWrapper
from scripts.collect_box_push_demos import collect_demos
from scripts.pretrain_box_push_bc import train_behavior_cloning
from scripts.watch_training_progress import newest_model_path


def test_expert_push_points_are_behind_box():
    expert = BoxPushHeuristicPolicy()
    box_pose = (0.0, 0.0, 0.0)
    zone_position = (0.7, 0.0)

    push_points = expert.push_points(box_pose, zone_position)

    assert push_points[0][0] < box_pose[0]
    assert push_points[1][0] < box_pose[0]


def test_expert_actions_are_normalized_and_deterministic():
    env = BoxPushEnv(render_mode="direct", difficulty="easy")
    expert = BoxPushHeuristicPolicy()
    try:
        observation, _ = env.reset()

        action_a = expert.predict(observation)
        action_b = expert.predict(observation)

        assert action_a.shape == (4,)
        assert np.all(action_a >= -1.0)
        assert np.all(action_a <= 1.0)
        np.testing.assert_allclose(action_a, action_b)
    finally:
        env.close()


def test_expert_improves_easy_box_distance():
    env = FlatActionWrapper(BoxPushEnv(render_mode="direct", difficulty="easy", max_steps=500))
    expert = BoxPushHeuristicPolicy()
    try:
        observation, _ = env.reset()
        initial_distance = env.unwrapped.box_zone_distance()
        for _ in range(220):
            observation, _, terminated, truncated, _ = env.step(expert.predict(observation))
            if terminated or truncated:
                break

        assert env.unwrapped.box_zone_distance() < initial_distance
    finally:
        env.close()


def test_demo_collection_writes_expected_npz(tmp_path):
    output_path = tmp_path / "box_push_demos.npz"

    collect_demos(
        steps=24,
        difficulties=["easy"],
        output_path=output_path,
        max_episode_steps=200,
        require_success=False,
        max_attempts_per_difficulty=10,
        randomization=0.0,
    )

    data = np.load(output_path)
    assert data["observations"].shape == (24, 18)
    assert data["actions"].shape == (24, 4)
    assert np.all(data["actions"] >= -1.0)
    assert np.all(data["actions"] <= 1.0)
    assert data["difficulties"].shape == (24,)


def test_tiny_behavior_cloning_reduces_loss_and_saves_model(tmp_path):
    demo_path = tmp_path / "demos.npz"
    model_dir = tmp_path / "models"
    log_dir = tmp_path / "runs"
    collect_demos(
        steps=32,
        difficulties=["easy"],
        output_path=demo_path,
        max_episode_steps=200,
        require_success=False,
        max_attempts_per_difficulty=10,
        randomization=0.0,
    )

    initial_loss, final_loss = train_behavior_cloning(
        demo_path=demo_path,
        model_dir=model_dir,
        log_dir=log_dir,
        epochs=2,
        batch_size=16,
        checkpoint_every=1,
        learning_rate=1e-3,
    )

    assert final_loss < initial_loss
    assert (model_dir / "box_push_bc_pretrained.zip").exists()
    model = PPO.load(model_dir / "box_push_bc_pretrained.zip")
    action, _ = model.predict(np.zeros(18, dtype=np.float32), deterministic=True)
    assert action.shape == (4,)


def test_expert_handles_cooperative_observations():
    env = BoxPushEnv(render_mode="direct", difficulty="coop_heavy")
    expert = BoxPushHeuristicPolicy()
    try:
        observation, _ = env.reset()
        action = expert.predict(observation)

        assert observation.shape == (18,)
        assert action.shape == (4,)
        assert np.all(action >= -1.0)
        assert np.all(action <= 1.0)
    finally:
        env.close()


@pytest.mark.parametrize("difficulty", ["coop_heavy", "coop_rotate", "coop_mixed"])
def test_expert_can_finish_cooperative_scenarios(difficulty):
    env = FlatActionWrapper(BoxPushEnv(render_mode="direct", difficulty=difficulty, max_steps=1400))
    expert = BoxPushHeuristicPolicy()
    try:
        observation, _ = env.reset()
        info = {}
        terminated = False
        for _ in range(env.unwrapped.max_steps):
            observation, _, terminated, truncated, info = env.step(expert.predict(observation))
            if terminated or truncated:
                break

        assert terminated is True
        assert info["box_in_zone"] is True
    finally:
        env.close()


def test_expert_makes_progress_on_coop_random_samples():
    expert = BoxPushHeuristicPolicy()
    for seed in range(5):
        env = FlatActionWrapper(BoxPushEnv(render_mode="direct", difficulty="coop_random", max_steps=1600))
        try:
            observation, _ = env.reset(seed=seed)
            initial_distance = env.unwrapped.box_zone_distance()
            info = {}
            for _ in range(env.unwrapped.max_steps):
                observation, _, terminated, truncated, info = env.step(expert.predict(observation))
                if terminated or truncated:
                    break

            assert info["box_zone_distance"] < initial_distance
        finally:
            env.close()


def test_checkpoint_watcher_can_select_bc_and_ppo_sources(tmp_path):
    bc_path = tmp_path / "bc_checkpoint_epoch_001.zip"
    prior_path = tmp_path / "prior_dagger_iter_001.zip"
    ppo_path = tmp_path / "box_push_ppo_100_steps.zip"
    bc_path.write_text("bc")
    prior_path.write_text("prior")
    ppo_path.write_text("ppo")

    assert newest_model_path(tmp_path, "bc") == bc_path
    assert newest_model_path(tmp_path, "prior") == prior_path
    assert newest_model_path(tmp_path, "ppo") == ppo_path
