import argparse
import math
import tkinter as tk

import numpy as np

from robot_lab_rl import BoxPushEnv, CircleRobotEnv


DEFAULT_SPEED_SCALE = 1.0
MIN_SPEED_SCALE = 0.1
MAX_SPEED_SCALE = 1.0
CANVAS_WIDTH = 900
CANVAS_HEIGHT = 450
CANVAS_PADDING = 28
UPDATE_MS = 16


def key_is_down(key_events: dict[str, bool], key: str) -> bool:
    return key_events.get(key, False)


def differential_keyboard_action(
    key_events: dict[str, bool],
    *,
    forward_key: str,
    backward_key: str,
    left_key: str,
    right_key: str,
    max_speed: float,
) -> tuple[float, float]:
    """Map keyboard state to left/right wheel speeds."""
    forward = key_is_down(key_events, forward_key)
    backward = key_is_down(key_events, backward_key)
    left = key_is_down(key_events, left_key)
    right = key_is_down(key_events, right_key)

    drive_speed = max_speed * 0.85
    turn_speed = max_speed * 0.65

    if forward and not backward:
        left_speed = drive_speed
        right_speed = drive_speed
    elif backward and not forward:
        left_speed = -drive_speed
        right_speed = -drive_speed
    else:
        left_speed = 0.0
        right_speed = 0.0

    if left and not right:
        left_speed -= turn_speed
        right_speed += turn_speed
    elif right and not left:
        left_speed += turn_speed
        right_speed -= turn_speed

    left_speed = float(np.clip(left_speed, -max_speed, max_speed))
    right_speed = float(np.clip(right_speed, -max_speed, max_speed))
    return left_speed, right_speed


class ManualDriveApp:
    def __init__(self, env: CircleRobotEnv) -> None:
        self.env = env
        self.root = tk.Tk()
        self.root.title("Robot Lab 2D Manual Drive")
        self.root.resizable(False, False)

        self.canvas = tk.Canvas(
            self.root,
            width=CANVAS_WIDTH,
            height=CANVAS_HEIGHT,
            bg="#8c8c8c",
            highlightthickness=0,
        )
        self.canvas.pack()

        self.speed_slider = tk.Scale(
            self.root,
            from_=MIN_SPEED_SCALE,
            to=MAX_SPEED_SCALE,
            resolution=0.05,
            orient=tk.HORIZONTAL,
            label="Speed multiplier (0.10 slow - 1.00 full speed)",
            length=CANVAS_WIDTH,
        )
        self.speed_slider.set(DEFAULT_SPEED_SCALE)
        self.speed_slider.pack(fill=tk.X)

        self.keys: dict[str, bool] = {}
        self.zone_item: int | None = None
        self.object_items: list[int] = []
        self.robot_items: list[dict[str, int]] = []
        self.coordinate_key_items: dict[str, int] = {}
        self.scale = min(
            (CANVAS_WIDTH - 2 * CANVAS_PADDING) / self.env.arena_width,
            (CANVAS_HEIGHT - 2 * CANVAS_PADDING) / self.env.arena_height,
        )
        self.running = True

        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.canvas.focus_set()

        self._draw_arena()
        self._create_zone_item()
        self._create_object_items()
        self._create_coordinate_key()
        self._create_robot_items()

    def run(self) -> None:
        self._tick()
        self.root.mainloop()

    def _on_key_press(self, event: tk.Event) -> None:
        self.keys[event.keysym.lower()] = True

    def _on_key_release(self, event: tk.Event) -> None:
        self.keys[event.keysym.lower()] = False

    def _on_close(self) -> None:
        self.running = False
        self.env.close()
        self.root.destroy()

    def _tick(self) -> None:
        if not self.running:
            return

        actions = self._read_keyboard_actions()
        self.env.step(actions)
        self._draw_zone()
        self._draw_objects()
        self._draw_robots()
        self._draw_coordinate_key()
        self.root.after(UPDATE_MS, self._tick)

    def _read_keyboard_actions(self) -> np.ndarray:
        actions = np.zeros((self.env.num_robots, 2), dtype=np.float32)
        max_speed = self.env.drive.max_wheel_speed * float(self.speed_slider.get())

        if self.env.num_robots >= 1:
            actions[0] = differential_keyboard_action(
                self.keys,
                forward_key="i",
                backward_key="k",
                left_key="j",
                right_key="l",
                max_speed=max_speed,
            )

        if self.env.num_robots >= 2:
            actions[1] = differential_keyboard_action(
                self.keys,
                forward_key="up",
                backward_key="down",
                left_key="left",
                right_key="right",
                max_speed=max_speed,
            )

        return actions

    def _create_zone_item(self) -> None:
        if not hasattr(self.env, "zone_position"):
            return
        self.zone_item = self.canvas.create_rectangle(
            0,
            0,
            0,
            0,
            fill="#5fd36f",
            outline="#116b21",
            width=3,
            stipple="gray25",
        )
        self._draw_zone()

    def _draw_zone(self) -> None:
        if self.zone_item is None or not hasattr(self.env, "zone_position"):
            return
        zone_x, zone_y = self.env.zone_position
        screen_x, screen_y = self._world_to_screen(zone_x, zone_y)
        half_width_px = self.env.zone_half_width * self.scale
        half_height_px = self.env.zone_half_height * self.scale
        self.canvas.coords(
            self.zone_item,
            screen_x - half_width_px,
            screen_y - half_height_px,
            screen_x + half_width_px,
            screen_y + half_height_px,
        )

    def _create_object_items(self) -> None:
        for obj in self.env.dynamic_objects:
            if obj.kind == "ball":
                item = self.canvas.create_oval(
                    0,
                    0,
                    0,
                    0,
                    fill="#f3c332",
                    outline="#7a5a00",
                    width=2,
                )
            else:
                item = self.canvas.create_polygon(
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    fill="#4f8fd9",
                    outline="#173b66",
                    width=2,
                )
            self.object_items.append(item)

        self._draw_objects()

    def _create_robot_items(self) -> None:
        radius_px = self.env.drive.robot_radius * self.scale
        for robot_index in range(self.env.num_robots):
            body = self.canvas.create_oval(
                -radius_px,
                -radius_px,
                radius_px,
                radius_px,
                fill="#d91515",
                outline="#6e0000",
                width=2,
            )
            arrow = self.canvas.create_line(
                0,
                0,
                0,
                0,
                fill="#111111",
                width=3,
                arrow=tk.LAST,
                arrowshape=(12, 14, 5),
            )
            label = self.canvas.create_text(
                0,
                0,
                text=f"P{robot_index}",
                fill="#111111",
                font=("Arial", 14, "bold"),
            )
            self.robot_items.append({"body": body, "arrow": arrow, "label": label})

        self._draw_robots()

    def _draw_arena(self) -> None:
        left, top = self._world_to_screen(
            -self.env.arena_width / 2.0,
            self.env.arena_height / 2.0,
        )
        right, bottom = self._world_to_screen(
            self.env.arena_width / 2.0,
            -self.env.arena_height / 2.0,
        )
        self.canvas.create_rectangle(
            left,
            top,
            right,
            bottom,
            outline="#222222",
            width=4,
        )
        self.canvas.create_text(
            (left + right) / 2.0,
            top + 16,
            text="2.0 m x 1.0 m arena",
            fill="#222222",
            font=("Arial", 12, "bold"),
        )

    def _create_coordinate_key(self) -> None:
        box_width = 230
        box_height = 24 + 22 * self.env.num_robots
        margin = 12
        left = CANVAS_WIDTH - box_width - margin
        top = CANVAS_HEIGHT - box_height - margin
        right = CANVAS_WIDTH - margin
        bottom = CANVAS_HEIGHT - margin

        background = self.canvas.create_rectangle(
            left,
            top,
            right,
            bottom,
            fill="#d8d8d8",
            outline="#222222",
            width=2,
        )
        title = self.canvas.create_text(
            left + 10,
            top + 10,
            text="Coordinates (m, deg)",
            anchor=tk.NW,
            fill="#111111",
            font=("Arial", 11, "bold"),
        )
        self.coordinate_key_items = {
            "background": background,
            "title": title,
        }
        for robot_index in range(self.env.num_robots):
            self.coordinate_key_items[f"robot_{robot_index}"] = self.canvas.create_text(
                left + 10,
                top + 32 + robot_index * 22,
                text="",
                anchor=tk.NW,
                fill="#111111",
                font=("Menlo", 11),
            )

        self._draw_coordinate_key()

    def _draw_coordinate_key(self) -> None:
        for robot_index, (x, y, yaw) in enumerate(self.env.robot_poses()):
            yaw_degrees = math.degrees(yaw)
            self.canvas.itemconfigure(
                self.coordinate_key_items[f"robot_{robot_index}"],
                text=f"P{robot_index}: x={x:+.2f}, y={y:+.2f}, yaw={yaw_degrees:+.0f}",
            )

    def _draw_objects(self) -> None:
        for item, obj in zip(self.object_items, self.env.dynamic_objects):
            screen_x, screen_y = self._world_to_screen(obj.x, obj.y)
            if obj.kind == "ball":
                radius_px = obj.radius * self.scale
                self.canvas.coords(
                    item,
                    screen_x - radius_px,
                    screen_y - radius_px,
                    screen_x + radius_px,
                    screen_y + radius_px,
                )
            else:
                corners = [
                    (-obj.half_width, -obj.half_height),
                    (obj.half_width, -obj.half_height),
                    (obj.half_width, obj.half_height),
                    (-obj.half_width, obj.half_height),
                ]
                screen_points: list[float] = []
                cos_a = math.cos(obj.angle)
                sin_a = math.sin(obj.angle)
                for local_x, local_y in corners:
                    world_x = obj.x + local_x * cos_a - local_y * sin_a
                    world_y = obj.y + local_x * sin_a + local_y * cos_a
                    corner_screen_x, corner_screen_y = self._world_to_screen(world_x, world_y)
                    screen_points.extend([corner_screen_x, corner_screen_y])
                self.canvas.coords(
                    item,
                    *screen_points,
                )

    def _draw_robots(self) -> None:
        radius_px = self.env.drive.robot_radius * self.scale
        arrow_length_px = radius_px * 1.65
        label_offset_px = radius_px * 1.9

        for robot_index, (x, y, yaw) in enumerate(self.env.robot_poses()):
            screen_x, screen_y = self._world_to_screen(x, y)
            direction_x = math.cos(yaw)
            direction_y = -math.sin(yaw)

            items = self.robot_items[robot_index]
            self.canvas.coords(
                items["body"],
                screen_x - radius_px,
                screen_y - radius_px,
                screen_x + radius_px,
                screen_y + radius_px,
            )
            self.canvas.coords(
                items["arrow"],
                screen_x,
                screen_y,
                screen_x + direction_x * arrow_length_px,
                screen_y + direction_y * arrow_length_px,
            )
            self.canvas.coords(
                items["label"],
                screen_x,
                screen_y - label_offset_px,
            )

        for item in self.coordinate_key_items.values():
            self.canvas.tag_raise(item)

    def _world_to_screen(self, x: float, y: float) -> tuple[float, float]:
        screen_x = CANVAS_WIDTH / 2.0 + x * self.scale
        screen_y = CANVAS_HEIGHT / 2.0 - y * self.scale
        return screen_x, screen_y


def main() -> None:
    parser = argparse.ArgumentParser(description="Manually drive abstract Pi-pucks in pure 2D.")
    parser.add_argument(
        "--task",
        choices=["sandbox", "box-push"],
        default="sandbox",
        help="Manual sandbox mode or the simple box-to-zone task.",
    )
    parser.add_argument("--robots", type=int, default=2, help="Number of circular robots in sandbox mode.")
    args = parser.parse_args()

    if args.task == "box-push":
        env = BoxPushEnv(render_mode="direct", max_steps=10_000_000)
    else:
        env = CircleRobotEnv(render_mode="direct", num_robots=args.robots, max_steps=10_000_000)
    observation, _ = env.reset()

    print("Manual 2D drive started.")
    print(f"Task: {args.task}")
    print("P0: I/K forward/backward, J/L turn.")
    print("P1: arrow up/down forward/backward, arrow left/right turn.")
    print("Speed: use the slider below the canvas.")
    print(f"Initial observation: {observation.round(3)}")

    app = ManualDriveApp(env)
    app.run()


if __name__ == "__main__":
    main()
