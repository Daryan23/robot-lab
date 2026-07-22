import numpy as np
import pytest

from robot_lab_rl import BoxPushEnv, BoxPushHeuristicPolicy
from robot_lab_rl.rl import FlatActionWrapper
from scripts.watch_training_progress import newest_model_path


@pytest.mark.parametrize("num_robots", [2, 3])
def test_expert_push_points_are_behind_box(num_robots):
    expert = BoxPushHeuristicPolicy()
    box_pose = (0.0, 0.0, 0.0)
    zone_position = (0.7, 0.0)

    push_points = expert.push_points(box_pose, zone_position, num_robots)

    assert len(push_points) == num_robots
    assert all(point[0] < box_pose[0] for point in push_points)


@pytest.mark.parametrize("num_robots", [2, 3])
def test_expert_actions_are_normalized_and_deterministic(num_robots):
    env = BoxPushEnv(render_mode="direct", difficulty="easy", num_robots=num_robots)
    expert = BoxPushHeuristicPolicy()
    try:
        observation, _ = env.reset()

        action_a = expert.predict(observation)
        action_b = expert.predict(observation)

        assert action_a.shape == (2 * num_robots,)
        assert np.all(action_a >= -1.0)
        assert np.all(action_a <= 1.0)
        np.testing.assert_allclose(action_a, action_b)
    finally:
        env.close()


def test_expert_improves_easy_box_distance():
    env = FlatActionWrapper(
        BoxPushEnv(render_mode="direct", difficulty="easy", max_steps=500, num_robots=2)
    )
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


@pytest.mark.parametrize("num_robots", [2, 3])
def test_expert_handles_cooperative_observations(num_robots):
    env = BoxPushEnv(render_mode="direct", difficulty="coop_heavy", num_robots=num_robots)
    expert = BoxPushHeuristicPolicy()
    try:
        observation, _ = env.reset()
        action = expert.predict(observation)

        assert observation.shape == (3 * num_robots + 12,)
        assert action.shape == (2 * num_robots,)
        assert np.all(action >= -1.0)
        assert np.all(action <= 1.0)
    finally:
        env.close()


@pytest.mark.parametrize("difficulty", ["coop_heavy", "coop_rotate", "coop_mixed"])
def test_expert_can_finish_cooperative_scenarios(difficulty):
    # The gravity-like cooperation makes the box move slower with a partial
    # team, so the scripted expert needs a larger step budget to finish.
    env = FlatActionWrapper(
        BoxPushEnv(render_mode="direct", difficulty=difficulty, max_steps=3000, num_robots=2)
    )
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
        env = FlatActionWrapper(
            BoxPushEnv(render_mode="direct", difficulty="coop_random", max_steps=1600, num_robots=2)
        )
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


def test_checkpoint_watcher_selects_newest_ppo_model(tmp_path):
    older_path = tmp_path / "box_push_ppo_100_steps.zip"
    newer_path = tmp_path / "box_push_ppo_200_steps.zip"
    older_path.write_text("older")
    newer_path.write_text("newer")
    # Ensure a distinct, later mtime for the newer checkpoint.
    import os
    import time

    now = time.time()
    os.utime(older_path, (now - 10, now - 10))
    os.utime(newer_path, (now, now))

    assert newest_model_path(tmp_path) == newer_path
