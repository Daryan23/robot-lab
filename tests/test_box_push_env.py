import numpy as np
import pytest

from robot_lab_rl import BoxPushEnv
from scripts.manual_drive import ManualDriveApp


def test_box_push_spawns_two_robots_one_box_and_one_zone():
    env = BoxPushEnv(render_mode="direct")
    try:
        observation, _ = env.reset()

        assert env.num_robots == 2
        assert len(env.robot_states) == 2
        assert len(env.dynamic_objects) == 1
        assert env.box.kind == "block"
        assert env.zone_position == pytest.approx((0.72, 0.0))
        assert observation.shape == (18,)
    finally:
        env.close()


def test_box_push_spaces_match_plan():
    env = BoxPushEnv(render_mode="direct")
    try:
        env.reset()

        assert env.action_space.shape == (2, 2)
        assert env.observation_space.shape == (18,)
    finally:
        env.close()


def test_box_push_reward_improves_when_box_moves_toward_zone():
    env = BoxPushEnv(render_mode="direct")
    try:
        env.reset()
        env.box.x = 0.0
        env.box.y = 0.0
        _, reward_without_progress, *_ = env.step(np.zeros((2, 2), dtype=np.float32))

        env.box.x = 0.0
        env.box.y = 0.0
        env.box.vx = 0.2
        env.box.vy = 0.0

        _, reward_with_progress, *_ = env.step(np.zeros((2, 2), dtype=np.float32))

        assert reward_with_progress > reward_without_progress
    finally:
        env.close()


def test_box_push_reward_improves_when_robots_move_toward_push_points():
    env = BoxPushEnv(render_mode="direct")
    try:
        env.reset()
        _, reward_without_setup_progress, *_ = env.step(np.zeros((2, 2), dtype=np.float32))

        push_points = env.push_points()
        env.robot_states = [
            (push_points[0][0] + 0.05, push_points[0][1], 0.0),
            (push_points[1][0] + 0.05, push_points[1][1], 0.0),
        ]
        _, reward_with_setup_progress, *_ = env.step(np.zeros((2, 2), dtype=np.float32))

        assert reward_with_setup_progress > reward_without_setup_progress
    finally:
        env.close()


def test_box_push_success_when_box_enters_zone():
    env = BoxPushEnv(render_mode="direct")
    try:
        env.reset()
        env.box.x, env.box.y = env.zone_position

        _, reward, terminated, truncated, info = env.step(np.zeros((2, 2), dtype=np.float32))

        assert terminated is True
        assert truncated is False
        assert reward > 1.0
        assert info["box_in_zone"] is True
    finally:
        env.close()


def test_box_push_does_not_succeed_when_box_only_touches_zone():
    env = BoxPushEnv(render_mode="direct")
    try:
        env.reset()
        zone_x, zone_y = env.zone_position
        env.box.x = zone_x - env.zone_half_width - env.box.half_width + 0.01
        env.box.y = zone_y

        _, reward, terminated, truncated, info = env.step(np.zeros((2, 2), dtype=np.float32))

        assert terminated is False
        assert truncated is False
        assert reward < 1.0
        assert info["box_in_zone"] is False
    finally:
        env.close()


def test_box_push_reports_alignment_contact_and_stuck_metrics():
    env = BoxPushEnv(render_mode="direct", difficulty="easy")
    try:
        env.reset()
        _, _, _, _, info = env.step(
            np.full((2, 2), env.drive.max_wheel_speed, dtype=np.float32)
        )

        assert 0.0 <= info["heading_alignment"] <= 1.0
        assert 0.0 <= info["behind_box_quality"] <= 1.0
        assert 0.0 <= info["contact_quality"] <= 1.0
        assert "team_robot_box_distance" in info
        assert "max_robot_box_distance" in info
        assert "both_robot_contact" in info
        assert "complementary_contact_quality" in info
        assert "contact_balance" in info
        assert "orientation_error" in info
        assert "box_zone_fit_error" in info
        assert info["no_progress"] is False
    finally:
        env.close()


def test_box_push_truncates_when_no_progress_is_made():
    env = BoxPushEnv(render_mode="direct", difficulty="easy")
    try:
        env.reset()
        truncated = False
        info = {}
        for _ in range(370):
            _, _, _, truncated, info = env.step(np.zeros((2, 2), dtype=np.float32))
            if truncated:
                break

        assert truncated is True
        assert info["no_progress"] is True
    finally:
        env.close()


def test_box_push_viewer_can_create_zone_and_box_items():
    env = BoxPushEnv(render_mode="direct")
    try:
        env.reset()
        app = ManualDriveApp(env)
        try:
            assert app.zone_item is not None
            assert len(app.object_items) == 1
        finally:
            app._on_close()
    finally:
        env.close()


@pytest.mark.parametrize("difficulty", ["coop_heavy", "coop_rotate", "coop_mixed", "coop_random"])
def test_cooperative_layouts_configure_larger_task_objects(difficulty):
    env = BoxPushEnv(render_mode="direct", difficulty=difficulty)
    try:
        observation, _ = env.reset()

        assert observation.shape == (18,)
        if difficulty == "coop_random":
            assert 0.70 <= env.box.mass <= 1.30
        else:
            assert env.box.mass >= 0.9
        assert env.box.half_width > 0.05
        assert env.layout.requires_cooperation is True
        assert env.zone_half_width == pytest.approx(env.layout.zone_half_width)
    finally:
        env.close()


def test_coop_random_samples_shape_mass_target_and_starts_within_bounds():
    env = BoxPushEnv(render_mode="direct", difficulty="coop_random")
    try:
        observation, _ = env.reset(seed=2)

        assert observation.shape == (18,)
        assert 0.07 <= env.box.half_width <= 0.15
        assert 0.035 <= env.box.half_height <= 0.08
        assert 0.70 <= env.box.mass <= 1.30
        assert -0.6 <= env.box.angle <= 0.6
        assert -0.35 <= env.target_angle <= 0.35
        assert -env.arena_width / 2.0 < env.box.x < env.arena_width / 2.0
        assert -env.arena_height / 2.0 < env.box.y < env.arena_height / 2.0
        zone_x, zone_y = env.zone_position
        assert -env.arena_width / 2.0 < zone_x < env.arena_width / 2.0
        assert -env.arena_height / 2.0 < zone_y < env.arena_height / 2.0
        for robot_x, robot_y, _ in env.robot_states:
            assert -env.arena_width / 2.0 < robot_x < env.arena_width / 2.0
            assert -env.arena_height / 2.0 < robot_y < env.arena_height / 2.0
    finally:
        env.close()


def test_coop_random_resamples_layouts_between_resets():
    env = BoxPushEnv(render_mode="direct", difficulty="coop_random")
    try:
        env.reset(seed=1)
        first = (env.box.half_width, env.box.half_height, env.box.mass, env.box.x, env.box.y, env.target_angle)
        env.reset(seed=2)
        second = (env.box.half_width, env.box.half_height, env.box.mass, env.box.x, env.box.y, env.target_angle)

        assert first != second
    finally:
        env.close()


def test_coop_rotate_requires_target_orientation_for_success():
    env = BoxPushEnv(render_mode="direct", difficulty="coop_rotate")
    try:
        env.reset()
        env.box.x, env.box.y = env.zone_position
        env.box.angle = env.target_angle + 0.8

        _, _, terminated, _, info = env.step(np.zeros((2, 2), dtype=np.float32))

        assert terminated is False
        assert info["box_in_zone"] is False

        env.box.x, env.box.y = env.zone_position
        env.box.angle = env.target_angle
        _, _, terminated, _, info = env.step(np.zeros((2, 2), dtype=np.float32))

        assert terminated is True
        assert info["box_in_zone"] is True
    finally:
        env.close()


def test_cooperative_contact_reward_beats_single_robot_contact():
    env = BoxPushEnv(render_mode="direct", difficulty="coop_heavy")
    try:
        env.reset()
        contact_x = env.box.x - env.box.half_width - env.drive.robot_radius * 0.8
        env.robot_states = [
            (contact_x, env.box.y + env.box.half_height * 0.8, 0.0),
            (env.box.x - 0.45, env.box.y + 0.30, 0.0),
        ]
        _, single_reward, *_ = env.step(np.zeros((2, 2), dtype=np.float32))

        env.reset()
        contact_x = env.box.x - env.box.half_width - env.drive.robot_radius * 0.8
        env.robot_states = [
            (contact_x, env.box.y + env.box.half_height * 0.8, 0.0),
            (contact_x, env.box.y - env.box.half_height * 0.8, 0.0),
        ]
        _, team_reward, *_ = env.step(np.zeros((2, 2), dtype=np.float32))

        assert team_reward > single_reward
    finally:
        env.close()


def test_heavy_object_push_scale_requires_two_contacts():
    env = BoxPushEnv(render_mode="direct", difficulty="coop_heavy")
    try:
        env.reset()
        contact_x = env.box.x - env.box.half_width - env.drive.robot_radius * 0.8
        env.robot_states = [
            (contact_x, env.box.y + env.box.half_height * 0.8, 0.0),
            (env.box.x - 0.45, env.box.y + 0.30, 0.0),
        ]
        single_scale = env._object_push_scale(env.box, 0)

        env.robot_states = [
            (contact_x, env.box.y + env.box.half_height * 0.8, 0.0),
            (contact_x, env.box.y - env.box.half_height * 0.8, 0.0),
        ]
        team_scale = env._object_push_scale(env.box, 0)

        assert team_scale > single_scale
    finally:
        env.close()
