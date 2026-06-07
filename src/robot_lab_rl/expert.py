from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ExpertConfig:
    wheel_distance: float = 0.055
    max_wheel_speed: float = 0.15
    robot_radius: float = 0.037
    box_half_width: float = 0.05
    box_half_height: float = 0.04
    push_point_extra_margin: float = 0.005
    push_lateral_offset: float = 0.035
    position_tolerance: float = 0.060
    align_tolerance: float = 0.40


class BoxPushHeuristicPolicy:
    """Decentralized geometric expert for the two-robot box-push task."""

    def __init__(self, config: ExpertConfig | None = None) -> None:
        self.config = config or ExpertConfig()

    def predict(self, observation: np.ndarray) -> np.ndarray:
        obs = np.asarray(observation, dtype=np.float32)
        if obs.shape not in {(11,), (18,)}:
            raise ValueError(f"Expected observation shape (11,) or (18,), got {obs.shape}")

        robot_poses = [(float(obs[0]), float(obs[1]), float(obs[2])), (float(obs[3]), float(obs[4]), float(obs[5]))]
        box_pose = (float(obs[6]), float(obs[7]), float(obs[8]))
        if obs.shape == (18,):
            zone_position = (float(obs[12]), float(obs[13]))
            target_angle = float(obs[14])
            box_half_width = float(obs[15])
            box_half_height = float(obs[16])
        else:
            zone_position = (float(obs[9]), float(obs[10]))
            target_angle = 0.0
            box_half_width = self.config.box_half_width
            box_half_height = self.config.box_half_height
        push_points = self.assigned_push_points(
            robot_poses,
            box_pose,
            zone_position,
            target_angle=target_angle,
            box_half_width=box_half_width,
            box_half_height=box_half_height,
        )

        actions = [
            self._robot_action(robot_pose, push_point, box_pose, zone_position)
            for robot_pose, push_point in zip(robot_poses, push_points)
        ]
        return np.asarray(actions, dtype=np.float32).reshape(4)

    def push_points(
        self,
        box_pose: tuple[float, float, float],
        zone_position: tuple[float, float],
        target_angle: float = 0.0,
        box_half_width: float | None = None,
        box_half_height: float | None = None,
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        box_x, box_y, box_angle = box_pose
        direction_x, direction_y = self._box_to_zone_direction(box_pose, zone_position)
        perpendicular_x = -direction_y
        perpendicular_y = direction_x
        half_width = self.config.box_half_width if box_half_width is None else box_half_width
        half_height = self.config.box_half_height if box_half_height is None else box_half_height
        behind_distance = (
            max(half_width, half_height)
            + self.config.robot_radius
            + self.config.push_point_extra_margin
        )
        base_x = box_x - direction_x * behind_distance
        base_y = box_y - direction_y * behind_distance
        lateral_offset = max(self.config.push_lateral_offset, min(half_height + 0.025, half_height * 1.1))
        angle_error = self._normalize_angle(target_angle - box_angle)
        if abs(angle_error) > 0.25 and half_width > half_height * 1.8:
            lateral_offset = min(half_width * 0.75, max(lateral_offset, half_height + 0.06))
        return (
            (base_x + perpendicular_x * lateral_offset, base_y + perpendicular_y * lateral_offset),
            (base_x - perpendicular_x * lateral_offset, base_y - perpendicular_y * lateral_offset),
        )

    def assigned_push_points(
        self,
        robot_poses: list[tuple[float, float, float]],
        box_pose: tuple[float, float, float],
        zone_position: tuple[float, float],
        target_angle: float = 0.0,
        box_half_width: float | None = None,
        box_half_height: float | None = None,
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        points = self.push_points(
            box_pose,
            zone_position,
            target_angle=target_angle,
            box_half_width=box_half_width,
            box_half_height=box_half_height,
        )
        robot_points = [(x, y) for x, y, _ in robot_poses]
        direct = self._assignment_distance(robot_points, points)
        swapped = (points[1], points[0])
        swapped_distance = self._assignment_distance(robot_points, swapped)
        return points if direct <= swapped_distance else swapped

    def _robot_action(
        self,
        robot_pose: tuple[float, float, float],
        push_point: tuple[float, float],
        box_pose: tuple[float, float, float],
        zone_position: tuple[float, float],
    ) -> tuple[float, float]:
        robot_x, robot_y, yaw = robot_pose
        box_x, box_y, _ = box_pose
        direction_x, direction_y = self._box_to_zone_direction(box_pose, zone_position)
        rel_x = robot_x - box_x
        rel_y = robot_y - box_y
        behind_depth = -(rel_x * direction_x + rel_y * direction_y)
        lateral_error = abs(rel_x * (-direction_y) + rel_y * direction_x)
        push_point_distance = math.hypot(robot_x - push_point[0], robot_y - push_point[1])
        is_behind_box = behind_depth > 0.05 and lateral_error < 0.14
        near_box = math.hypot(robot_x - box_x, robot_y - box_y) < 0.18

        if not is_behind_box or push_point_distance > self.config.position_tolerance:
            return self._drive_to_target(robot_pose, push_point, max_forward=0.9)

        target_yaw = math.atan2(box_y - robot_y, box_x - robot_x)
        heading_error = self._normalize_angle(target_yaw - yaw)
        if abs(heading_error) > self.config.align_tolerance:
            return self._turn_in_place(heading_error)

        forward = 1.0 if near_box else 0.65
        turn = float(np.clip(heading_error / 0.8, -0.35, 0.35))
        return self._forward_turn_to_wheels(forward, turn)

    def _drive_to_target(
        self,
        robot_pose: tuple[float, float, float],
        target: tuple[float, float],
        max_forward: float,
    ) -> tuple[float, float]:
        robot_x, robot_y, yaw = robot_pose
        target_x, target_y = target
        distance = math.hypot(target_x - robot_x, target_y - robot_y)
        desired_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = self._normalize_angle(desired_yaw - yaw)
        forward = float(np.clip(distance / 0.18, 0.0, max_forward))
        if abs(heading_error) > math.pi / 2.0:
            forward *= 0.15
        turn = float(np.clip(heading_error / 1.0, -1.0, 1.0))
        return self._forward_turn_to_wheels(forward, turn)

    def _turn_in_place(self, heading_error: float) -> tuple[float, float]:
        turn = float(np.clip(heading_error / 1.0, -1.0, 1.0))
        return -turn, turn

    def _forward_turn_to_wheels(self, forward: float, turn: float) -> tuple[float, float]:
        left = float(np.clip(forward - turn, -1.0, 1.0))
        right = float(np.clip(forward + turn, -1.0, 1.0))
        return left, right

    @staticmethod
    def _box_to_zone_direction(
        box_pose: tuple[float, float, float],
        zone_position: tuple[float, float],
    ) -> tuple[float, float]:
        dx = zone_position[0] - box_pose[0]
        dy = zone_position[1] - box_pose[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-8:
            return 1.0, 0.0
        return dx / distance, dy / distance

    @staticmethod
    def _assignment_distance(
        robot_points: list[tuple[float, float]],
        push_points: tuple[tuple[float, float], tuple[float, float]],
    ) -> float:
        return sum(
            math.hypot(robot_x - push_x, robot_y - push_y)
            for (robot_x, robot_y), (push_x, push_y) in zip(robot_points, push_points)
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi
