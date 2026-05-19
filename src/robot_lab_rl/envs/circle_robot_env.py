from __future__ import annotations

import math
from dataclasses import dataclass

import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
from gymnasium import spaces


@dataclass(frozen=True)
class DifferentialDriveConfig:
    """Physical constants for the abstract circular Pi-puck."""

    wheel_distance: float = 0.055
    max_wheel_speed: float = 0.15
    robot_radius: float = 0.037
    robot_height: float = 0.025


@dataclass
class DynamicObject2D:
    kind: str
    x: float
    y: float
    vx: float
    vy: float
    mass: float
    radius: float = 0.0
    half_width: float = 0.0
    half_height: float = 0.0
    angle: float = 0.0
    omega: float = 0.0

    @property
    def inertia(self) -> float:
        if self.kind == "ball":
            return 0.5 * self.mass * self.radius * self.radius
        width = self.half_width * 2.0
        height = self.half_height * 2.0
        return self.mass * (width * width + height * height) / 12.0


def differential_drive_velocity(
    left_wheel_speed: float,
    right_wheel_speed: float,
    wheel_distance: float,
) -> tuple[float, float]:
    """Return linear and angular velocity from left/right wheel speeds."""
    linear_velocity = (left_wheel_speed + right_wheel_speed) / 2.0
    angular_velocity = (right_wheel_speed - left_wheel_speed) / wheel_distance
    return linear_velocity, angular_velocity


class CircleRobotEnv(gym.Env):
    """A tiny 2D PyBullet world with circular differential-drive robots.

    Each robot is represented as a short cylinder. We constrain it to the XY
    plane by directly controlling its planar velocity and yaw rate.
    """

    metadata = {"render_modes": ["human", "direct"], "render_fps": 60}

    def __init__(
        self,
        render_mode: str = "direct",
        arena_width: float = 2.0,
        arena_height: float = 1.0,
        time_step: float = 1.0 / 60.0,
        max_steps: int = 600,
        num_robots: int = 2,
    ) -> None:
        super().__init__()
        if render_mode not in self.metadata["render_modes"]:
            raise ValueError(f"Unsupported render_mode: {render_mode}")
        if num_robots < 1:
            raise ValueError("num_robots must be at least 1.")

        self.render_mode = render_mode
        self.arena_width = arena_width
        self.arena_height = arena_height
        self.world_size = max(arena_width, arena_height) / 2.0
        self.time_step = time_step
        self.max_steps = max_steps
        self.num_robots = num_robots
        self.drive = DifferentialDriveConfig()

        self.action_space = spaces.Box(
            low=-self.drive.max_wheel_speed,
            high=self.drive.max_wheel_speed,
            shape=(num_robots, 2),
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(
            low=np.tile(
                np.array(
                    [
                        -self.arena_width / 2.0,
                        -self.arena_height / 2.0,
                        -math.pi,
                    ],
                    dtype=np.float32,
                ),
                num_robots,
            ),
            high=np.tile(
                np.array(
                    [
                        self.arena_width / 2.0,
                        self.arena_height / 2.0,
                        math.pi,
                    ],
                    dtype=np.float32,
                ),
                num_robots,
            ),
            dtype=np.float32,
        )

        self.client_id: int | None = None
        self.robot_ids: list[int] = []
        self.robot_states: list[tuple[float, float, float]] = []
        self.robot_velocities: list[tuple[float, float]] = []
        self.dynamic_objects: list[DynamicObject2D] = []
        self.step_count = 0

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        self._connect()
        self._build_world()
        self.step_count = 0
        return self._get_observation(), {}

    def step(self, action):
        if not self.robot_ids:
            raise RuntimeError("Call reset() before step().")

        clipped_action = np.clip(
            np.asarray(action, dtype=np.float32),
            self.action_space.low,
            self.action_space.high,
        )

        for robot_index, wheel_speeds in enumerate(clipped_action):
            self.apply_robot_wheel_speeds(
                robot_index,
                float(wheel_speeds[0]),
                float(wheel_speeds[1]),
            )
        self._step_dynamic_objects()
        p.stepSimulation(physicsClientId=self.client_id)

        self.step_count += 1
        observation = self._get_observation()
        terminated = False
        truncated = self.step_count >= self.max_steps
        reward = -0.001
        info = {"poses": self.robot_poses()}
        return observation, reward, terminated, truncated, info

    def apply_robot_wheel_speeds(
        self,
        robot_index: int,
        left_wheel_speed: float,
        right_wheel_speed: float,
    ) -> None:
        """Apply one robot's wheel speeds in strict 2D kinematics."""
        if self.client_id is None or not self.robot_ids:
            raise RuntimeError("Call reset() before controlling robots.")

        linear_velocity, angular_velocity = differential_drive_velocity(
            left_wheel_speed,
            right_wheel_speed,
            self.drive.wheel_distance,
        )
        x, y, yaw = self.robot_states[robot_index]
        yaw = self._normalize_angle(yaw + angular_velocity * self.time_step)
        x += linear_velocity * math.cos(yaw) * self.time_step
        y += linear_velocity * math.sin(yaw) * self.time_step
        x = float(
            np.clip(
                x,
                -self.arena_width / 2.0 + self.drive.robot_radius,
                self.arena_width / 2.0 - self.drive.robot_radius,
            )
        )
        y = float(
            np.clip(
                y,
                -self.arena_height / 2.0 + self.drive.robot_radius,
                self.arena_height / 2.0 - self.drive.robot_radius,
            )
        )

        previous_x, previous_y, _ = self.robot_states[robot_index]
        self.robot_states[robot_index] = (x, y, yaw)
        self.robot_velocities[robot_index] = (
            (x - previous_x) / self.time_step,
            (y - previous_y) / self.time_step,
        )
        self._sync_robot_body(robot_index)

    def set_robot_wheel_speeds(
        self,
        robot_index: int,
        left_wheel_speed: float,
        right_wheel_speed: float,
    ) -> None:
        """Backward-compatible alias for applying 2D wheel-speed motion."""
        self.apply_robot_wheel_speeds(robot_index, left_wheel_speed, right_wheel_speed)

    def _sync_robot_body(self, robot_index: int) -> None:
        assert self.client_id is not None
        x, y, yaw = self.robot_states[robot_index]
        p.resetBasePositionAndOrientation(
            self.robot_ids[robot_index],
            [x, y, self.drive.robot_height / 2.0],
            p.getQuaternionFromEuler([0.0, 0.0, yaw]),
            physicsClientId=self.client_id,
        )
        p.resetBaseVelocity(
            self.robot_ids[robot_index],
            linearVelocity=[0.0, 0.0, 0.0],
            angularVelocity=[0.0, 0.0, 0.0],
            physicsClientId=self.client_id,
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def close(self) -> None:
        if self.client_id is not None:
            p.disconnect(physicsClientId=self.client_id)
            self.client_id = None
            self.robot_ids = []
            self.robot_states = []
            self.robot_velocities = []
            self.dynamic_objects = []

    def _connect(self) -> None:
        if self.client_id is not None:
            return
        connection_mode = p.GUI if self.render_mode == "human" else p.DIRECT
        self.client_id = p.connect(connection_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.setTimeStep(self.time_step, physicsClientId=self.client_id)
        p.setGravity(0.0, 0.0, 0.0, physicsClientId=self.client_id)
        if self.render_mode == "human":
            p.resetDebugVisualizerCamera(
                cameraDistance=2.2,
                cameraYaw=0,
                cameraPitch=-89,
                cameraTargetPosition=[0.0, 0.0, 0.0],
                physicsClientId=self.client_id,
            )

    def _build_world(self) -> None:
        assert self.client_id is not None
        p.resetSimulation(physicsClientId=self.client_id)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.setTimeStep(self.time_step, physicsClientId=self.client_id)
        p.setGravity(0.0, 0.0, 0.0, physicsClientId=self.client_id)
        self._create_floor()

        collision_shape = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=self.drive.robot_radius,
            height=self.drive.robot_height,
            physicsClientId=self.client_id,
        )
        self.robot_ids = []
        start_positions = self._start_positions()
        self.robot_states = [(x, y, 0.0) for x, y in start_positions]
        self.robot_velocities = [(0.0, 0.0) for _ in start_positions]
        self.dynamic_objects = self._create_random_dynamic_objects()

        for robot_index in range(self.num_robots):
            visual_shape = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=self.drive.robot_radius,
                length=self.drive.robot_height,
                rgbaColor=[0.9, 0.05, 0.04, 1.0],
                physicsClientId=self.client_id,
            )
            robot_id = p.createMultiBody(
                baseMass=0.1,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=[
                    self.robot_states[robot_index][0],
                    self.robot_states[robot_index][1],
                    self.drive.robot_height / 2.0,
                ],
                baseOrientation=p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
                physicsClientId=self.client_id,
            )
            p.changeDynamics(
                robot_id,
                -1,
                lateralFriction=1.0,
                linearDamping=0.0,
                angularDamping=0.0,
                physicsClientId=self.client_id,
            )
            self.robot_ids.append(robot_id)

        self._draw_robot_labels()

    def _create_random_dynamic_objects(self) -> list[DynamicObject2D]:
        rng = np.random.default_rng(7)
        objects: list[DynamicObject2D] = []
        candidates: list[DynamicObject2D] = [
            DynamicObject2D("block", -0.55, 0.18, 0.0, 0.0, mass=0.35, half_width=0.055, half_height=0.055),
            DynamicObject2D("block", 0.05, 0.25, 0.0, 0.0, mass=0.45, half_width=0.07, half_height=0.045),
            DynamicObject2D("block", 0.55, -0.18, 0.0, 0.0, mass=0.35, half_width=0.05, half_height=0.065),
            DynamicObject2D("ball", -0.25, -0.22, 0.0, 0.0, mass=0.18, radius=0.04),
            DynamicObject2D("ball", 0.32, 0.02, 0.0, 0.0, mass=0.18, radius=0.04),
            DynamicObject2D("ball", 0.72, 0.24, 0.0, 0.0, mass=0.14, radius=0.035),
        ]
        for candidate in candidates:
            candidate.x += float(rng.uniform(-0.03, 0.03))
            candidate.y += float(rng.uniform(-0.03, 0.03))
            objects.append(candidate)
        return objects

    def _step_dynamic_objects(self) -> None:
        for obj in self.dynamic_objects:
            obj.x += obj.vx * self.time_step
            obj.y += obj.vy * self.time_step
            obj.angle = self._normalize_angle(obj.angle + obj.omega * self.time_step)

        for robot_index, (robot_x, robot_y, _) in enumerate(self.robot_states):
            robot_vx, robot_vy = self.robot_velocities[robot_index]
            for obj in self.dynamic_objects:
                self._resolve_robot_object_collision(robot_x, robot_y, robot_vx, robot_vy, obj)

        for _ in range(2):
            for index, obj_a in enumerate(self.dynamic_objects):
                for obj_b in self.dynamic_objects[index + 1 :]:
                    self._resolve_object_object_collision(obj_a, obj_b)

        for obj in self.dynamic_objects:
            self._clip_dynamic_object_to_arena(obj)
            damping = 0.985 if obj.kind == "ball" else 0.94
            angular_damping = 0.97 if obj.kind == "ball" else 0.90
            obj.vx *= damping
            obj.vy *= damping
            obj.omega *= angular_damping
            if abs(obj.vx) < 0.002:
                obj.vx = 0.0
            if abs(obj.vy) < 0.002:
                obj.vy = 0.0
            if abs(obj.omega) < 0.02:
                obj.omega = 0.0

    def _resolve_robot_object_collision(
        self,
        robot_x: float,
        robot_y: float,
        robot_vx: float,
        robot_vy: float,
        obj: DynamicObject2D,
    ) -> None:
        closest_x, closest_y = self._closest_point_on_object(robot_x, robot_y, obj)
        dx = robot_x - closest_x
        dy = robot_y - closest_y
        distance = math.hypot(dx, dy)
        if distance >= self.drive.robot_radius:
            return

        if distance < 1e-8:
            normal_x, normal_y = self._fallback_push_normal(robot_x, robot_y, obj.x, obj.y)
        else:
            normal_x = dx / distance
            normal_y = dy / distance

        penetration = self.drive.robot_radius - distance
        obj.x -= normal_x * penetration
        obj.y -= normal_y * penetration

        push_velocity = robot_vx * (-normal_x) + robot_vy * (-normal_y)
        if push_velocity > 0.0:
            normal_velocity = obj.vx * (-normal_x) + obj.vy * (-normal_y)
            target_normal_velocity = min(push_velocity * 0.9, math.hypot(robot_vx, robot_vy))
            delta_normal_velocity = max(0.0, target_normal_velocity - normal_velocity)
            delta_vx = -normal_x * delta_normal_velocity
            delta_vy = -normal_y * delta_normal_velocity
            obj.vx += delta_vx
            obj.vy += delta_vy
            self._cap_object_speed(obj, max(push_velocity, math.hypot(robot_vx, robot_vy)))

            if obj.kind == "block":
                contact_x = closest_x
                contact_y = closest_y
                impulse_x = delta_vx * obj.mass
                impulse_y = delta_vy * obj.mass
                torque_impulse = self._cross_2d(contact_x - obj.x, contact_y - obj.y, impulse_x, impulse_y)
                obj.omega += torque_impulse / max(obj.inertia, 1e-6)
                obj.omega = float(np.clip(obj.omega, -6.0, 6.0))

    def _resolve_object_object_collision(self, obj_a: DynamicObject2D, obj_b: DynamicObject2D) -> None:
        overlap, normal_x, normal_y, penetration = self._object_overlap(obj_a, obj_b)
        if not overlap:
            return

        total_mass = obj_a.mass + obj_b.mass
        obj_a_share = obj_b.mass / total_mass
        obj_b_share = obj_a.mass / total_mass
        obj_a.x -= normal_x * penetration * obj_a_share
        obj_a.y -= normal_y * penetration * obj_a_share
        obj_b.x += normal_x * penetration * obj_b_share
        obj_b.y += normal_y * penetration * obj_b_share

        rel_vx = obj_b.vx - obj_a.vx
        rel_vy = obj_b.vy - obj_a.vy
        separating_speed = rel_vx * normal_x + rel_vy * normal_y
        if separating_speed < 0.0:
            restitution = 0.35 if "ball" in (obj_a.kind, obj_b.kind) else 0.12
            impulse = -(1.0 + restitution) * separating_speed / (1.0 / obj_a.mass + 1.0 / obj_b.mass)
            obj_a.vx -= impulse * normal_x / obj_a.mass
            obj_a.vy -= impulse * normal_y / obj_a.mass
            obj_b.vx += impulse * normal_x / obj_b.mass
            obj_b.vy += impulse * normal_y / obj_b.mass

    def _object_overlap(
        self,
        obj_a: DynamicObject2D,
        obj_b: DynamicObject2D,
    ) -> tuple[bool, float, float, float]:
        if obj_a.kind == "ball" and obj_b.kind == "ball":
            dx = obj_b.x - obj_a.x
            dy = obj_b.y - obj_a.y
            distance = math.hypot(dx, dy)
            min_distance = obj_a.radius + obj_b.radius
            if distance >= min_distance:
                return False, 0.0, 0.0, 0.0
            if distance < 1e-8:
                return True, 1.0, 0.0, min_distance
            return True, dx / distance, dy / distance, min_distance - distance

        if obj_a.kind == "ball" and obj_b.kind == "block":
            closest_x, closest_y = self._closest_point_on_object(obj_a.x, obj_a.y, obj_b)
            dx = closest_x - obj_a.x
            dy = closest_y - obj_a.y
            distance = math.hypot(dx, dy)
            if distance >= obj_a.radius:
                return False, 0.0, 0.0, 0.0
            if distance < 1e-8:
                normal_x, normal_y = self._fallback_push_normal(obj_b.x, obj_b.y, obj_a.x, obj_a.y)
                return True, normal_x, normal_y, obj_a.radius
            return True, dx / distance, dy / distance, obj_a.radius - distance

        if obj_a.kind == "block" and obj_b.kind == "ball":
            overlap, normal_x, normal_y, penetration = self._object_overlap(obj_b, obj_a)
            return overlap, -normal_x, -normal_y, penetration

        dx = obj_b.x - obj_a.x
        dy = obj_b.y - obj_a.y
        overlap_x = obj_a.half_width + obj_b.half_width - abs(dx)
        overlap_y = obj_a.half_height + obj_b.half_height - abs(dy)
        if overlap_x <= 0.0 or overlap_y <= 0.0:
            return False, 0.0, 0.0, 0.0
        if overlap_x < overlap_y:
            normal_x = 1.0 if dx >= 0.0 else -1.0
            return True, normal_x, 0.0, overlap_x
        normal_y = 1.0 if dy >= 0.0 else -1.0
        return True, 0.0, normal_y, overlap_y

    def _closest_point_on_object(
        self,
        x: float,
        y: float,
        obj: DynamicObject2D,
    ) -> tuple[float, float]:
        if obj.kind == "ball":
            dx = x - obj.x
            dy = y - obj.y
            distance = math.hypot(dx, dy)
            if distance < 1e-8:
                return obj.x, obj.y
            return obj.x + dx / distance * obj.radius, obj.y + dy / distance * obj.radius
        local_x, local_y = self._world_to_object_local(x, y, obj)
        closest_local_x = float(np.clip(local_x, -obj.half_width, obj.half_width))
        closest_local_y = float(np.clip(local_y, -obj.half_height, obj.half_height))
        return self._object_local_to_world(closest_local_x, closest_local_y, obj)

    def _world_to_object_local(self, x: float, y: float, obj: DynamicObject2D) -> tuple[float, float]:
        dx = x - obj.x
        dy = y - obj.y
        cos_a = math.cos(obj.angle)
        sin_a = math.sin(obj.angle)
        return (
            dx * cos_a + dy * sin_a,
            -dx * sin_a + dy * cos_a,
        )

    def _object_local_to_world(self, local_x: float, local_y: float, obj: DynamicObject2D) -> tuple[float, float]:
        cos_a = math.cos(obj.angle)
        sin_a = math.sin(obj.angle)
        return (
            obj.x + local_x * cos_a - local_y * sin_a,
            obj.y + local_x * sin_a + local_y * cos_a,
        )

    def _clip_dynamic_object_to_arena(self, obj: DynamicObject2D) -> None:
        margin_x, margin_y = self._object_world_extents(obj)
        min_x = -self.arena_width / 2.0 + margin_x
        max_x = self.arena_width / 2.0 - margin_x
        min_y = -self.arena_height / 2.0 + margin_y
        max_y = self.arena_height / 2.0 - margin_y

        if obj.x < min_x:
            obj.x = min_x
            obj.vx = abs(obj.vx) * 0.25
        elif obj.x > max_x:
            obj.x = max_x
            obj.vx = -abs(obj.vx) * 0.25
        if obj.y < min_y:
            obj.y = min_y
            obj.vy = abs(obj.vy) * 0.25
        elif obj.y > max_y:
            obj.y = max_y
            obj.vy = -abs(obj.vy) * 0.25
            obj.omega *= 0.5

    def _object_world_extents(self, obj: DynamicObject2D) -> tuple[float, float]:
        if obj.kind == "ball":
            return obj.radius, obj.radius
        cos_a = abs(math.cos(obj.angle))
        sin_a = abs(math.sin(obj.angle))
        return (
            obj.half_width * cos_a + obj.half_height * sin_a,
            obj.half_width * sin_a + obj.half_height * cos_a,
        )

    def _cap_object_speed(self, obj: DynamicObject2D, max_speed: float) -> None:
        speed = math.hypot(obj.vx, obj.vy)
        allowed_speed = max(0.02, max_speed)
        if speed > allowed_speed:
            scale = allowed_speed / speed
            obj.vx *= scale
            obj.vy *= scale

    @staticmethod
    def _cross_2d(ax: float, ay: float, bx: float, by: float) -> float:
        return ax * by - ay * bx

    @staticmethod
    def _fallback_push_normal(x_a: float, y_a: float, x_b: float, y_b: float) -> tuple[float, float]:
        dx = x_a - x_b
        dy = y_a - y_b
        distance = math.hypot(dx, dy)
        if distance < 1e-8:
            return 1.0, 0.0
        return dx / distance, dy / distance

    def _create_floor(self) -> None:
        assert self.client_id is not None
        floor_half_extents = [self.arena_width / 2.0, self.arena_height / 2.0, 0.005]
        floor_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=floor_half_extents,
            rgbaColor=[0.55, 0.55, 0.55, 1.0],
            physicsClientId=self.client_id,
        )
        floor_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=floor_half_extents,
            physicsClientId=self.client_id,
        )
        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=floor_collision,
            baseVisualShapeIndex=floor_visual,
            basePosition=[0.0, 0.0, -floor_half_extents[2]],
            physicsClientId=self.client_id,
        )

    def _start_positions(self) -> list[tuple[float, float]]:
        spacing = 0.22
        offset = spacing * (self.num_robots - 1) / 2.0
        return [(index * spacing - offset, 0.0) for index in range(self.num_robots)]

    def _draw_robot_labels(self) -> None:
        assert self.client_id is not None
        for robot_index, robot_id in enumerate(self.robot_ids):
            marker_z = self.drive.robot_height + 0.01
            arrow_tip = self.drive.robot_radius * 1.55
            arrow_base = self.drive.robot_radius * 0.15
            arrow_head_back = self.drive.robot_radius * 0.95
            arrow_head_width = self.drive.robot_radius * 0.35

            p.addUserDebugText(
                f"P{robot_index}",
                textPosition=[0.0, 0.0, self.drive.robot_height + 0.04],
                textColorRGB=[0.0, 0.0, 0.0],
                textSize=1.2,
                parentObjectUniqueId=robot_id,
                physicsClientId=self.client_id,
            )
            p.addUserDebugLine(
                [arrow_base, 0.0, marker_z],
                [arrow_tip, 0.0, marker_z],
                lineColorRGB=[0.0, 0.0, 0.0],
                lineWidth=3.0,
                parentObjectUniqueId=robot_id,
                physicsClientId=self.client_id,
            )
            p.addUserDebugLine(
                [arrow_tip, 0.0, marker_z],
                [arrow_head_back, arrow_head_width, marker_z],
                lineColorRGB=[0.0, 0.0, 0.0],
                lineWidth=3.0,
                parentObjectUniqueId=robot_id,
                physicsClientId=self.client_id,
            )
            p.addUserDebugLine(
                [arrow_tip, 0.0, marker_z],
                [arrow_head_back, -arrow_head_width, marker_z],
                lineColorRGB=[0.0, 0.0, 0.0],
                lineWidth=3.0,
                parentObjectUniqueId=robot_id,
                physicsClientId=self.client_id,
            )

    def add_box_obstacle(
        self,
        position: tuple[float, float],
        half_extents: tuple[float, float, float],
        color: tuple[float, float, float, float],
    ) -> int:
        assert self.client_id is not None
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=color,
            physicsClientId=self.client_id,
        )
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            physicsClientId=self.client_id,
        )
        return p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[position[0], position[1], half_extents[2]],
            physicsClientId=self.client_id,
        )

    def _get_observation(self) -> np.ndarray:
        return np.array(self.robot_poses(), dtype=np.float32).flatten()

    def robot_poses(self) -> list[tuple[float, float, float]]:
        return list(self.robot_states)

    def robot_pose(self, robot_index: int) -> tuple[float, float, float]:
        return self.robot_states[robot_index]
