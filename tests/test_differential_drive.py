import math

import pytest
import pybullet as p

from robot_lab_rl import CircleRobotEnv
from robot_lab_rl.envs.circle_robot_env import differential_drive_velocity
from scripts.manual_drive import differential_keyboard_action


def test_equal_wheel_speeds_drive_straight():
    linear_velocity, angular_velocity = differential_drive_velocity(0.1, 0.1, 0.05)

    assert linear_velocity == pytest.approx(0.1)
    assert angular_velocity == pytest.approx(0.0)


def test_opposite_wheel_speeds_rotate_in_place():
    linear_velocity, angular_velocity = differential_drive_velocity(-0.1, 0.1, 0.05)

    assert linear_velocity == pytest.approx(0.0)
    assert angular_velocity == pytest.approx(4.0)


def test_robot_moves_forward_when_both_wheels_are_positive():
    env = CircleRobotEnv(render_mode="direct", num_robots=2)
    try:
        initial_observation, _ = env.reset()
        action = [[0.1, 0.1], [0.0, 0.0]]

        observation = initial_observation
        for _ in range(30):
            observation, *_ = env.step(action)

        assert observation[0] > initial_observation[0]
    finally:
        env.close()


def test_robot_body_stays_flat_in_2d():
    env = CircleRobotEnv(render_mode="direct", num_robots=1)
    try:
        env.reset()
        for _ in range(30):
            env.step([[0.1, -0.1]])

        position, orientation = p.getBasePositionAndOrientation(
            env.robot_ids[0],
            physicsClientId=env.client_id,
        )
        roll, pitch, _ = p.getEulerFromQuaternion(orientation)

        assert position[2] == pytest.approx(env.drive.robot_height / 2.0)
        assert roll == pytest.approx(0.0)
        assert pitch == pytest.approx(0.0)
    finally:
        env.close()


def test_default_arena_is_two_by_one_meters():
    env = CircleRobotEnv(render_mode="direct", num_robots=1)
    try:
        env.reset()

        assert env.arena_width == pytest.approx(2.0)
        assert env.arena_height == pytest.approx(1.0)
        assert env.observation_space.low.tolist() == pytest.approx([-1.0, -0.5, -math.pi])
        assert env.observation_space.high.tolist() == pytest.approx([1.0, 0.5, math.pi])
    finally:
        env.close()


def test_robot_is_clipped_inside_arena_bounds():
    env = CircleRobotEnv(render_mode="direct", num_robots=1)
    try:
        env.reset()
        env.robot_states[0] = (0.0, env.arena_height / 2.0 - env.drive.robot_radius / 2.0, math.pi / 2.0)
        for _ in range(60):
            env.step([[0.15, 0.15]])

        _, y, _ = env.robot_pose(0)
        assert y <= env.arena_height / 2.0 - env.drive.robot_radius
    finally:
        env.close()


def test_reset_creates_movable_blocks_and_balls():
    env = CircleRobotEnv(render_mode="direct", num_robots=1)
    try:
        env.reset()

        kinds = [obj.kind for obj in env.dynamic_objects]
        assert kinds.count("block") >= 3
        assert kinds.count("ball") >= 3
    finally:
        env.close()


def test_robot_pushes_block_forward():
    env = CircleRobotEnv(render_mode="direct", num_robots=1)
    try:
        env.reset()
        block = next(obj for obj in env.dynamic_objects if obj.kind == "block")
        block.x = 0.0
        block.y = 0.0
        block.vx = 0.0
        block.vy = 0.0
        env.robot_states[0] = (-0.10, 0.0, 0.0)

        initial_block_x = block.x
        for _ in range(45):
            env.step([[0.15, 0.15]])

        assert block.x > initial_block_x
    finally:
        env.close()


def test_off_center_push_rotates_block():
    env = CircleRobotEnv(render_mode="direct", num_robots=1)
    try:
        env.reset()
        block = next(obj for obj in env.dynamic_objects if obj.kind == "block")
        env.dynamic_objects = [block]
        block.x = 0.0
        block.y = 0.0
        block.vx = 0.0
        block.vy = 0.0
        block.angle = 0.0
        block.omega = 0.0
        env.robot_states[0] = (-0.13, 0.04, 0.0)

        for _ in range(80):
            env.step([[0.15, 0.15]])

        assert abs(block.angle) > 0.05
        assert abs(block.omega) > 0.01
    finally:
        env.close()


def test_pushed_ball_does_not_exceed_robot_push_speed():
    env = CircleRobotEnv(render_mode="direct", num_robots=1)
    try:
        env.reset()
        ball = next(obj for obj in env.dynamic_objects if obj.kind == "ball")
        env.dynamic_objects = [ball]
        ball.x = 0.0
        ball.y = 0.0
        ball.vx = 0.0
        ball.vy = 0.0
        env.robot_states[0] = (-0.12, 0.0, 0.0)

        max_ball_speed = 0.0
        for _ in range(80):
            env.step([[0.15, 0.15]])
            max_ball_speed = max(max_ball_speed, math.hypot(ball.vx, ball.vy))

        assert max_ball_speed <= 0.15
    finally:
        env.close()


def test_keyboard_forward_drives_both_wheels_forward():
    left_speed, right_speed = differential_keyboard_action(
        {"i": True},
        forward_key="i",
        backward_key="k",
        left_key="j",
        right_key="l",
        max_speed=0.15,
    )

    assert left_speed == pytest.approx(right_speed)
    assert left_speed > 0


def test_keyboard_left_turn_uses_differential_wheel_speeds():
    left_speed, right_speed = differential_keyboard_action(
        {"j": True},
        forward_key="i",
        backward_key="k",
        left_key="j",
        right_key="l",
        max_speed=0.15,
    )

    assert left_speed < 0
    assert right_speed > 0
