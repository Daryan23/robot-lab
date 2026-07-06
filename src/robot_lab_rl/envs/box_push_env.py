from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from gymnasium import spaces

from robot_lab_rl.envs.circle_robot_env import CircleRobotEnv, DynamicObject2D


@dataclass(frozen=True)
class BoxPushLayout:
    robot_positions: tuple[tuple[float, float], tuple[float, float]]
    box_position: tuple[float, float]
    zone_position: tuple[float, float]
    box_angle: float = 0.0
    target_angle: float = 0.0
    box_mass: float = 0.45
    box_half_width: float = 0.05
    box_half_height: float = 0.04
    zone_half_width: float = 0.12
    zone_half_height: float = 0.10
    orientation_tolerance: float | None = None
    requires_cooperation: bool = False


LAYOUTS: dict[str, BoxPushLayout] = {
    "easy": BoxPushLayout(
        robot_positions=((0.34, -0.035), (0.34, 0.035)),
        box_position=(0.50, 0.0),
        zone_position=(0.72, 0.0),
    ),
    "medium": BoxPushLayout(
        robot_positions=((-0.08, -0.11), (-0.08, 0.11)),
        box_position=(0.16, 0.0),
        zone_position=(0.72, 0.0),
    ),
    "full": BoxPushLayout(
        robot_positions=((-0.68, -0.14), (-0.68, 0.14)),
        box_position=(-0.18, 0.0),
        zone_position=(0.72, 0.0),
    ),
    "coop_heavy": BoxPushLayout(
        robot_positions=((-0.72, -0.19), (-0.72, 0.19)),
        box_position=(-0.20, 0.0),
        zone_position=(0.50, 0.0),
        box_mass=1.20,
        box_half_width=0.105,
        box_half_height=0.070,
        zone_half_width=0.24,
        zone_half_height=0.16,
        requires_cooperation=True,
    ),
    "coop_rotate": BoxPushLayout(
        robot_positions=((-0.70, -0.22), (-0.70, 0.22)),
        box_position=(-0.18, 0.0),
        zone_position=(0.46, 0.0),
        box_angle=0.45,
        target_angle=0.0,
        box_mass=0.90,
        box_half_width=0.135,
        box_half_height=0.040,
        zone_half_width=0.40,
        zone_half_height=0.30,
        orientation_tolerance=0.50,
        requires_cooperation=True,
    ),
    "coop_mixed": BoxPushLayout(
        robot_positions=((-0.72, -0.28), (-0.58, 0.23)),
        box_position=(-0.18, -0.10),
        zone_position=(0.42, 0.12),
        box_angle=-0.25,
        target_angle=0.25,
        box_mass=0.95,
        box_half_width=0.115,
        box_half_height=0.055,
        zone_half_width=0.42,
        zone_half_height=0.36,
        orientation_tolerance=0.45,
        requires_cooperation=True,
    ),
    "coop_random": BoxPushLayout(
        robot_positions=((-0.72, -0.20), (-0.72, 0.20)),
        box_position=(-0.18, 0.0),
        zone_position=(0.48, 0.0),
        box_mass=1.0,
        box_half_width=0.11,
        box_half_height=0.055,
        zone_half_width=0.36,
        zone_half_height=0.28,
        orientation_tolerance=0.50,
        requires_cooperation=True,
    ),
}

DIFFICULTIES = tuple(LAYOUTS)


class BoxPushEnv(CircleRobotEnv):
    """Two robots, one movable box, one fixed target zone."""

    def __init__(
        self,
        render_mode: str = "direct",
        arena_width: float = 2.0,
        arena_height: float = 1.0,
        time_step: float = 1.0 / 60.0,
        max_steps: int = 800,
        zone_half_width: float = 0.12,
        zone_half_height: float = 0.10,
        difficulty: str = "full",
        randomization: float = 0.0,
    ) -> None:
        if difficulty not in LAYOUTS:
            raise ValueError(f"Unsupported difficulty: {difficulty}")
        if randomization < 0.0:
            raise ValueError("randomization must be non-negative.")
        layout = LAYOUTS[difficulty]
        super().__init__(
            render_mode=render_mode,
            arena_width=arena_width,
            arena_height=arena_height,
            time_step=time_step,
            max_steps=max_steps,
            num_robots=2,
        )
        self.difficulty = difficulty
        self.layout = layout
        self.base_zone_half_width = zone_half_width
        self.base_zone_half_height = zone_half_height
        self.zone_half_width = layout.zone_half_width if difficulty.startswith("coop_") else zone_half_width
        self.zone_half_height = layout.zone_half_height if difficulty.startswith("coop_") else zone_half_height
        self.zone_radius = max(self.zone_half_width, self.zone_half_height)
        self.zone_position = LAYOUTS[difficulty].zone_position
        self.target_angle = layout.target_angle
        self.orientation_tolerance = layout.orientation_tolerance
        self.randomization = randomization
        self.previous_box_zone_distance = 0.0
        self.previous_orientation_error = 0.0
        self.previous_push_setup_distance = 0.0
        self.previous_team_robot_box_distance = 0.0
        self.best_box_zone_distance = 0.0
        self.last_useful_progress_step = 0
        observation_low = np.array(
            [
                -arena_width / 2.0,
                -arena_height / 2.0,
                -math.pi,
                -arena_width / 2.0,
                -arena_height / 2.0,
                -math.pi,
                -arena_width / 2.0,
                -arena_height / 2.0,
                -math.pi,
                -1.0,
                -1.0,
                -8.0,
                -arena_width / 2.0,
                -arena_height / 2.0,
                -math.pi,
                0.0,
                0.0,
                0.0,
            ],
            dtype=np.float32,
        )
        observation_high = np.array(
            [
                arena_width / 2.0,
                arena_height / 2.0,
                math.pi,
                arena_width / 2.0,
                arena_height / 2.0,
                math.pi,
                arena_width / 2.0,
                arena_height / 2.0,
                math.pi,
                1.0,
                1.0,
                8.0,
                arena_width / 2.0,
                arena_height / 2.0,
                math.pi,
                0.25,
                0.25,
                2.0,
            ],
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(
            low=observation_low,
            high=observation_high,
            dtype=np.float32,
        )

    def _before_build_world(self) -> None:
        self.layout = self._sample_random_layout() if self.difficulty == "coop_random" else LAYOUTS[self.difficulty]
        self.zone_half_width = (
            self.layout.zone_half_width if self.difficulty.startswith("coop_") else self.base_zone_half_width
        )
        self.zone_half_height = (
            self.layout.zone_half_height if self.difficulty.startswith("coop_") else self.base_zone_half_height
        )
        self.zone_radius = max(self.zone_half_width, self.zone_half_height)
        self.zone_position = self.layout.zone_position
        self.target_angle = self.layout.target_angle
        self.orientation_tolerance = self.layout.orientation_tolerance

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        observation, info = super().reset(seed=seed, options=options)
        del observation
        self.previous_box_zone_distance = self.box_zone_distance()
        self.previous_orientation_error = self.orientation_error()
        self.previous_push_setup_distance = self.push_setup_distance()
        self.previous_team_robot_box_distance = self.team_robot_box_distance()
        self.best_box_zone_distance = self.previous_box_zone_distance
        self.last_useful_progress_step = 0
        return self._get_observation(), info

    def step(self, action):
        if not self.robot_ids:
            raise RuntimeError("Call reset() before step().")

        previous_distance = self.box_zone_distance()
        previous_orientation_error = self.orientation_error()
        previous_push_setup_distance = self.push_setup_distance()
        previous_team_robot_box_distance = self.team_robot_box_distance()
        observation, _, _, truncated, info = super().step(action)
        del observation

        current_distance = self.box_zone_distance()
        current_push_setup_distance = self.push_setup_distance()
        current_robot_box_distance = self.robot_box_distance()
        current_team_robot_box_distance = self.team_robot_box_distance()
        current_max_robot_box_distance = self.max_robot_box_distance()
        box_progress = previous_distance - current_distance
        orientation_progress = previous_orientation_error - self.orientation_error()
        approach_progress = previous_push_setup_distance - current_push_setup_distance
        robot_box_progress = previous_team_robot_box_distance - current_team_robot_box_distance
        box_velocity_to_zone = self.box_velocity_toward_zone()
        heading_alignment = self.heading_alignment()
        behind_box_quality = self.behind_box_quality()
        contact_quality = self.contact_quality()
        both_robot_contact = self.both_robots_in_contact()
        complementary_contact = self.complementary_contact_quality()
        contact_balance = self.contact_balance()
        action_array = np.asarray(action, dtype=np.float32)
        action_penalty = float(np.mean(np.abs(action_array)) / self.drive.max_wheel_speed) * 0.002
        backward_box_penalty = abs(min(0.0, box_velocity_to_zone)) * 0.8
        useful_progress = (
            box_progress > 0.001
            or orientation_progress > 0.002
            or robot_box_progress > 0.001
            or approach_progress > 0.001
            or contact_quality > 0.03
        )
        if useful_progress:
            self.last_useful_progress_step = self.step_count
        if current_distance < self.best_box_zone_distance - 0.01:
            self.best_box_zone_distance = min(self.best_box_zone_distance, current_distance)

        reward_terms = {
            "box_progress": box_progress * 120.0,
            "approach_progress": approach_progress * 12.0,
            "robot_box_progress": robot_box_progress * 5.0,
            "box_velocity_to_zone": max(0.0, box_velocity_to_zone) * 0.5,
            "heading_alignment": heading_alignment * 0.001,
            "behind_box_quality": behind_box_quality * 0.002,
            "contact_quality": contact_quality * 0.01,
            "orientation_progress": max(0.0, orientation_progress)
            * (50.0 if self.orientation_tolerance is not None else 8.0),
            "complementary_contact": complementary_contact
            * (0.03 if self.layout.requires_cooperation else 0.008),
            "contact_balance": contact_balance
            * (0.02 if self.layout.requires_cooperation else 0.004),
            "both_robot_contact": 0.05 if both_robot_contact else 0.0,
            "time_penalty": -0.01,
            "action_penalty": -action_penalty,
            "backward_box_penalty": -backward_box_penalty,
            "spread_penalty": -current_max_robot_box_distance * 0.002,
        }
        if self.layout.requires_cooperation and contact_balance < 0.2:
            reward_terms["contact_balance_penalty"] = -0.015
        terminated = self.box_in_zone()
        no_progress_limit = {
            "easy": 360,
            "medium": 420,
            "full": 520,
            "coop_heavy": 700,
            "coop_rotate": 820,
            "coop_mixed": 820,
            "coop_random": 900,
        }[self.difficulty]
        no_progress = self.step_count - self.last_useful_progress_step >= no_progress_limit
        truncated = truncated or (no_progress and not terminated)
        if terminated:
            remaining_episode_fraction = 1.0 - self.step_count / max(self.max_steps, 1)
            reward_terms["success_bonus"] = 80.0 + 20.0 * remaining_episode_fraction
        elif no_progress:
            reward_terms["stall_penalty"] = -8.0
        elif truncated:
            reward_terms["timeout_penalty"] = -15.0
        reward = float(sum(reward_terms.values()))

        info = {
            **info,
            "box_zone_distance": current_distance,
            "box_in_zone": terminated,
            "zone_position": self.zone_position,
            "robot_box_distance": current_robot_box_distance,
            "team_robot_box_distance": current_team_robot_box_distance,
            "max_robot_box_distance": current_max_robot_box_distance,
            "push_setup_distance": current_push_setup_distance,
            "box_zone_fit_error": self.box_zone_fit_error(),
            "orientation_error": self.orientation_error(),
            "orientation_progress": orientation_progress,
            "box_progress": box_progress,
            "approach_progress": approach_progress,
            "robot_box_progress": robot_box_progress,
            "box_velocity_to_zone": box_velocity_to_zone,
            "heading_alignment": heading_alignment,
            "behind_box_quality": behind_box_quality,
            "contact_quality": contact_quality,
            "both_robot_contact": both_robot_contact,
            "complementary_contact_quality": complementary_contact,
            "contact_balance": contact_balance,
            "useful_progress": useful_progress,
            "no_progress": no_progress,
            "reward_terms": reward_terms,
        }
        self.previous_box_zone_distance = current_distance
        self.previous_orientation_error = self.orientation_error()
        self.previous_push_setup_distance = current_push_setup_distance
        self.previous_team_robot_box_distance = current_team_robot_box_distance
        return self._get_observation(), reward, terminated, truncated, info

    @property
    def box(self) -> DynamicObject2D:
        return self.dynamic_objects[0]

    def box_zone_distance(self) -> float:
        zone_x, zone_y = self.zone_position
        return math.hypot(self.box.x - zone_x, self.box.y - zone_y)

    def box_in_zone(self) -> bool:
        zone_x, zone_y = self.zone_position
        fits_position = all(
            abs(corner_x - zone_x) <= self.zone_half_width
            and abs(corner_y - zone_y) <= self.zone_half_height
            for corner_x, corner_y in self.box_corners()
        )
        if not fits_position:
            return False
        if self.orientation_tolerance is None:
            return True
        return self.orientation_error() <= self.orientation_tolerance

    def box_zone_fit_error(self) -> float:
        zone_x, zone_y = self.zone_position
        return max(
            0.0,
            max(
                max(
                    abs(corner_x - zone_x) - self.zone_half_width,
                    abs(corner_y - zone_y) - self.zone_half_height,
                )
                for corner_x, corner_y in self.box_corners()
            ),
        )

    def orientation_error(self) -> float:
        return abs(self._normalize_angle(self.target_angle - self.box.angle))

    def box_corners(self) -> list[tuple[float, float]]:
        return [
            self._object_local_to_world(local_x, local_y, self.box)
            for local_x, local_y in [
                (-self.box.half_width, -self.box.half_height),
                (self.box.half_width, -self.box.half_height),
                (self.box.half_width, self.box.half_height),
                (-self.box.half_width, self.box.half_height),
            ]
        ]

    def push_setup_distance(self) -> float:
        push_points = self.push_points()
        robot_points = [(x, y) for x, y, _ in self.robot_states]
        direct = self._assignment_distance(robot_points, push_points)
        swapped = self._assignment_distance(robot_points, (push_points[1], push_points[0]))
        return min(direct, swapped)

    def robot_box_distance(self) -> float:
        return min(self.robot_box_distances())

    def team_robot_box_distance(self) -> float:
        return float(np.mean(self.robot_box_distances()))

    def max_robot_box_distance(self) -> float:
        return max(self.robot_box_distances())

    def robot_box_distances(self) -> list[float]:
        distances = []
        for robot_x, robot_y, _ in self.robot_states:
            closest_x, closest_y = self._closest_point_on_object(robot_x, robot_y, self.box)
            distances.append(math.hypot(robot_x - closest_x, robot_y - closest_y))
        return distances

    def both_robots_in_contact(self) -> bool:
        contact_limit = self.drive.robot_radius + 0.012
        return all(distance <= contact_limit for distance in self.robot_box_distances())

    def contact_balance(self) -> float:
        distances = self.robot_box_distances()
        contact_limit = self.drive.robot_radius + 0.012
        contacts = [float(np.clip(1.0 - distance / contact_limit, 0.0, 1.0)) for distance in distances]
        return float(min(contacts))

    def complementary_contact_quality(self) -> float:
        contact_limit = self.drive.robot_radius + 0.012
        contact_points = []
        for robot_x, robot_y, _ in self.robot_states:
            closest_x, closest_y = self._closest_point_on_object(robot_x, robot_y, self.box)
            if math.hypot(robot_x - closest_x, robot_y - closest_y) <= contact_limit:
                local_x, local_y = self._world_to_object_local(closest_x, closest_y, self.box)
                contact_points.append((local_x, local_y))
        if len(contact_points) < 2:
            return 0.0
        lateral_separation = abs(contact_points[0][1] - contact_points[1][1])
        longitudinal_separation = abs(contact_points[0][0] - contact_points[1][0])
        useful_span = max(self.box.half_height * 1.4, self.box.half_width * 0.6, 1e-6)
        return float(np.clip(max(lateral_separation, longitudinal_separation) / useful_span, 0.0, 1.0))

    def box_velocity_toward_zone(self) -> float:
        direction_x, direction_y = self.box_to_zone_direction()
        return self.box.vx * direction_x + self.box.vy * direction_y

    def heading_alignment(self) -> float:
        push_points = self.assigned_push_points()
        alignments = []
        for (robot_x, robot_y, yaw), (target_x, target_y) in zip(self.robot_states, push_points):
            if math.hypot(robot_x - target_x, robot_y - target_y) < 0.04:
                target_x, target_y = self.box.x, self.box.y
            desired_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
            alignments.append((math.cos(self._normalize_angle(desired_yaw - yaw)) + 1.0) / 2.0)
        return float(np.mean(alignments))

    def behind_box_quality(self) -> float:
        direction_x, direction_y = self.box_to_zone_direction()
        qualities = []
        for robot_x, robot_y, _ in self.robot_states:
            rel_x = robot_x - self.box.x
            rel_y = robot_y - self.box.y
            behind_depth = -(rel_x * direction_x + rel_y * direction_y)
            lateral_error = abs(self._cross_2d(direction_x, direction_y, rel_x, rel_y))
            depth_quality = float(np.clip(behind_depth / 0.18, 0.0, 1.0))
            lateral_quality = float(np.clip(1.0 - lateral_error / 0.18, 0.0, 1.0))
            qualities.append(depth_quality * lateral_quality)
        return float(np.mean(qualities))

    def contact_quality(self) -> float:
        direction_x, direction_y = self.box_to_zone_direction()
        qualities = []
        for robot_index, (robot_x, robot_y, _) in enumerate(self.robot_states):
            closest_x, closest_y = self._closest_point_on_object(robot_x, robot_y, self.box)
            contact_distance = math.hypot(robot_x - closest_x, robot_y - closest_y)
            if contact_distance > self.drive.robot_radius + 0.012:
                qualities.append(0.0)
                continue
            robot_vx, robot_vy = self.robot_velocities[robot_index]
            push_alignment = max(0.0, robot_vx * direction_x + robot_vy * direction_y) / max(
                self.drive.max_wheel_speed,
                1e-6,
            )
            qualities.append(self.behind_box_quality() * float(np.clip(push_alignment, 0.0, 1.0)))
        return float(np.mean(qualities))

    def push_points(self) -> tuple[tuple[float, float], tuple[float, float]]:
        direction_x, direction_y = self.box_to_zone_direction()
        perpendicular_x = -direction_y
        perpendicular_y = direction_x
        box_margin = max(self.box.half_width, self.box.half_height)
        behind_distance = box_margin + self.drive.robot_radius + 0.035
        lateral_offset = min(self.box.half_height + 0.03, max(0.07, self.box.half_height * 0.95))
        base_x = self.box.x - direction_x * behind_distance
        base_y = self.box.y - direction_y * behind_distance
        return (
            (base_x + perpendicular_x * lateral_offset, base_y + perpendicular_y * lateral_offset),
            (base_x - perpendicular_x * lateral_offset, base_y - perpendicular_y * lateral_offset),
        )

    def assigned_push_points(self) -> tuple[tuple[float, float], tuple[float, float]]:
        push_points = self.push_points()
        robot_points = [(x, y) for x, y, _ in self.robot_states]
        direct = self._assignment_distance(robot_points, push_points)
        swapped_points = (push_points[1], push_points[0])
        swapped = self._assignment_distance(robot_points, swapped_points)
        return push_points if direct <= swapped else swapped_points

    def box_to_zone_direction(self) -> tuple[float, float]:
        zone_x, zone_y = self.zone_position
        dx = zone_x - self.box.x
        dy = zone_y - self.box.y
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

    def _start_positions(self) -> list[tuple[float, float]]:
        return [
            self._jitter_point(position, self.randomization)
            for position in self.layout.robot_positions
        ]

    def _create_random_dynamic_objects(self) -> list[DynamicObject2D]:
        layout = self.layout
        box_x, box_y = self._jitter_point(layout.box_position, self.randomization)
        zone_jitter = self.randomization * 0.5
        self.zone_position = self._jitter_point(layout.zone_position, zone_jitter)
        angle_jitter = float(self.np_random.uniform(-self.randomization * 2.0, self.randomization * 2.0))
        return [
            DynamicObject2D(
                "block",
                x=box_x,
                y=box_y,
                vx=0.0,
                vy=0.0,
                mass=layout.box_mass,
                half_width=layout.box_half_width,
                half_height=layout.box_half_height,
                angle=self._normalize_angle(layout.box_angle + angle_jitter),
            )
        ]

    def _sample_random_layout(self) -> BoxPushLayout:
        box_half_width = float(self.np_random.uniform(0.07, 0.15))
        box_half_height = float(self.np_random.uniform(0.035, 0.08))
        box_mass = float(self.np_random.uniform(0.70, 1.30))
        box_x = float(self.np_random.uniform(-0.24, -0.12))
        box_y = float(self.np_random.uniform(-0.06, 0.06))
        box_angle = float(self.np_random.uniform(-0.6, 0.6))
        target_angle = float(self.np_random.uniform(-0.35, 0.35))
        zone_x = float(self.np_random.uniform(0.42, 0.54))
        zone_y = float(self.np_random.uniform(-0.08, 0.08))
        direction_x = zone_x - box_x
        direction_y = zone_y - box_y
        direction_length = max(math.hypot(direction_x, direction_y), 1e-6)
        direction_x /= direction_length
        direction_y /= direction_length
        perpendicular_x = -direction_y
        perpendicular_y = direction_x
        behind_distance = max(box_half_width, box_half_height) + self.drive.robot_radius + 0.36
        lateral_offset = float(self.np_random.uniform(0.16, 0.22))
        base_x = box_x - direction_x * behind_distance
        base_y = box_y - direction_y * behind_distance
        robot_positions = (
            self._clip_robot_start(
                base_x + perpendicular_x * lateral_offset,
                base_y + perpendicular_y * lateral_offset,
            ),
            self._clip_robot_start(
                base_x - perpendicular_x * lateral_offset,
                base_y - perpendicular_y * lateral_offset,
            ),
        )
        zone_half_width = float(np.clip(box_half_width + 0.30, 0.38, 0.48))
        zone_half_height = float(np.clip(box_half_height + 0.27, 0.31, 0.40))
        return BoxPushLayout(
            robot_positions=robot_positions,
            box_position=(box_x, box_y),
            zone_position=(zone_x, zone_y),
            box_angle=box_angle,
            target_angle=target_angle,
            box_mass=box_mass,
            box_half_width=box_half_width,
            box_half_height=box_half_height,
            zone_half_width=zone_half_width,
            zone_half_height=zone_half_height,
            orientation_tolerance=0.55,
            requires_cooperation=True,
        )

    def _clip_robot_start(self, x: float, y: float) -> tuple[float, float]:
        return (
            float(np.clip(x, -self.arena_width / 2.0 + 0.08, self.arena_width / 2.0 - 0.08)),
            float(np.clip(y, -self.arena_height / 2.0 + 0.08, self.arena_height / 2.0 - 0.08)),
        )

    def _object_push_scale(self, obj: DynamicObject2D, robot_index: int) -> float:
        del robot_index
        if obj is not self.box or not self.layout.requires_cooperation:
            return 1.0
        contacts = sum(
            distance <= self.drive.robot_radius + 0.012
            for distance in self.robot_box_distances()
        )
        assigned_points = self.assigned_push_points()
        near_assigned_points = all(
            math.hypot(robot_x - point_x, robot_y - point_y) <= 0.09
            for (robot_x, robot_y, _), (point_x, point_y) in zip(self.robot_states, assigned_points)
        )
        if contacts < 2:
            if near_assigned_points:
                return 0.65
            return 0.28
        return 0.65 + 0.35 * self.complementary_contact_quality()

    def _jitter_point(self, point: tuple[float, float], amount: float) -> tuple[float, float]:
        if amount <= 0.0:
            return point
        x = float(point[0] + self.np_random.uniform(-amount, amount))
        y = float(point[1] + self.np_random.uniform(-amount, amount))
        x = float(np.clip(x, -self.arena_width / 2.0 + 0.08, self.arena_width / 2.0 - 0.08))
        y = float(np.clip(y, -self.arena_height / 2.0 + 0.08, self.arena_height / 2.0 - 0.08))
        return x, y

    def _get_observation(self) -> np.ndarray:
        robot_values = np.array(self.robot_poses(), dtype=np.float32).flatten()
        box_values = np.array(
            [self.box.x, self.box.y, self.box.angle, self.box.vx, self.box.vy, self.box.omega],
            dtype=np.float32,
        )
        zone_values = np.array([*self.zone_position, self.target_angle], dtype=np.float32)
        object_values = np.array([self.box.half_width, self.box.half_height, self.box.mass], dtype=np.float32)
        return np.concatenate([robot_values, box_values, zone_values, object_values]).astype(np.float32)
