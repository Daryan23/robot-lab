import json
import math
import random
import threading
import time
import urllib.request

import paho.mqtt.client as mqtt
from pipuck.pipuck import PiPuck

BROKER = "192.168.178.43"
PORT = 1883
ROBOT_ID = 32
ZONES_URL = f"http://{BROKER}:3000/danger_zones.json"

WORKSPACE_X = (0.1, 1.9)
WORKSPACE_Y = (0.1, 0.9)
BOUNDARY_MARGIN = 0.10
ZONE_MARGIN = 0.08
LOOKAHEAD = 0.20
DIFF_PIVOT = math.radians(50)
DIFF_DEAD = math.radians(10)
RECOVER_DIFF_TOLERANCE = math.radians(20)

FORWARD_SPEED = 600
STEER_DELTA = 350
HARD_PIVOT_SPEED = 450
IR_SOFT = 600
IR_HARD = 1500
POSE_TIMEOUT = 5.0
OTHER_ROBOT_MARGIN = 0.18
OTHER_ROBOT_LOOKAHEAD_MARGIN = 0.15
OTHER_ROBOT_TIMEOUT = 3.0
TICK = 0.05
RUN_DURATION = 60.0
LOG_INTERVAL = 0.5

pose_lock = threading.Lock()
latest_pose = {"x": None, "y": None, "theta": None, "in_danger": False, "t": 0.0}

others_lock = threading.Lock()
other_robots = {}


def load_zones():
    try:
        with urllib.request.urlopen(ZONES_URL, timeout=3) as resp:
            data = json.loads(resp.read().decode())
        zones = data.get("zones", [])
        print(f"loaded {len(zones)} danger zone(s)")
        return zones
    except Exception as exc:
        print(f"could not load danger zones ({exc}); continuing without them")
        return []


def point_in_circle(x, y, zone, margin):
    dx = x - zone["center"]["x"]
    dy = y - zone["center"]["y"]
    return math.hypot(dx, dy) <= zone["radius"] + margin


def point_in_polygon(x, y, zone, margin):
    pts = zone["points"]
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]["x"], pts[i]["y"]
        xj, yj = pts[j]["x"], pts[j]["y"]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    if inside:
        return True
    if margin <= 0:
        return False
    min_d2 = float("inf")
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]["x"], pts[i]["y"]
        xj, yj = pts[j]["x"], pts[j]["y"]
        ex, ey = xi - xj, yi - yj
        seg_len2 = ex * ex + ey * ey
        t = 0.0 if seg_len2 == 0 else max(0.0, min(1.0, ((x - xj) * ex + (y - yj) * ey) / seg_len2))
        px, py = xj + t * ex, yj + t * ey
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < min_d2:
            min_d2 = d2
        j = i
    return min_d2 <= margin * margin


def point_in_any_zone(x, y, zones, margin):
    for z in zones:
        if z.get("type") == "circle" and point_in_circle(x, y, z, margin):
            return True
        if z.get("type") == "polygon" and point_in_polygon(x, y, z, margin):
            return True
    return False


def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("robot_pos/all")


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        return
    if not isinstance(data, dict):
        return
    now_t = time.time()
    self_key = str(ROBOT_ID)
    others_update = {}
    for rid, entry in data.items():
        if not isinstance(entry, dict):
            continue
        pos = entry.get("position")
        if not (isinstance(pos, (list, tuple)) and len(pos) >= 2):
            continue
        try:
            ox = float(pos[0])
            oy = float(pos[1])
        except (TypeError, ValueError):
            continue
        if str(rid) == self_key:
            try:
                theta = math.radians(float(entry.get("angle", 0.0)))
            except (TypeError, ValueError):
                continue
            in_danger = bool(entry.get("in_danger", False))
            with pose_lock:
                latest_pose.update(x=ox, y=oy, theta=theta, in_danger=in_danger, t=now_t)
        else:
            others_update[str(rid)] = (ox, oy, now_t)
    if others_update:
        with others_lock:
            other_robots.update(others_update)


def fresh_other_robots():
    cutoff = time.time() - OTHER_ROBOT_TIMEOUT
    with others_lock:
        return [(rid, x, y) for rid, (x, y, t) in other_robots.items() if t >= cutoff]


def nearest_other(x, y):
    best = None
    best_d = float("inf")
    for rid, ox, oy in fresh_other_robots():
        d = math.hypot(x - ox, y - oy)
        if d < best_d:
            best_d = d
            best = (rid, ox, oy, d)
    return best


def get_pose():
    with pose_lock:
        if latest_pose["x"] is None or time.time() - latest_pose["t"] > POSE_TIMEOUT:
            return None
        return (latest_pose["x"], latest_pose["y"], latest_pose["theta"], latest_pose["in_danger"])


def near_boundary(x, y):
    xmin, xmax = WORKSPACE_X
    ymin, ymax = WORKSPACE_Y
    return (
        x < xmin + BOUNDARY_MARGIN
        or x > xmax - BOUNDARY_MARGIN
        or y < ymin + BOUNDARY_MARGIN
        or y > ymax - BOUNDARY_MARGIN
    )


def heading_to_center(x, y, theta):
    cx = 0.5 * (WORKSPACE_X[0] + WORKSPACE_X[1])
    cy = 0.5 * (WORKSPACE_Y[0] + WORKSPACE_Y[1])
    target = math.atan2(cy - y, cx - x)
    return (target - theta + math.pi) % (2 * math.pi) - math.pi


def ir_front_max(ir):
    return max(ir[0], ir[7])


def ir_turn_sign(ir):
    return -1 if (ir[0] + ir[1]) > (ir[7] + ir[6]) else 1


def drive(pipuck, forward, steer):
    left = max(-1000, min(1000, forward + steer))
    right = max(-1000, min(1000, forward - steer))
    pipuck.epuck.set_motor_speeds(left, right)


zones = load_zones()

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()

pipuck = PiPuck(epuck_version=2)

start = time.time()
walk_state = "STRAIGHT"
walk_state_until = start + random.uniform(2.0, 4.5)
walk_turn_steer = 0
last_log = 0.0

try:
    while time.time() - start < RUN_DURATION:
        now = time.time()
        ir = pipuck.epuck.ir_reflected
        ir_front = ir_front_max(ir)

        if ir_front > IR_HARD:
            sign = ir_turn_sign(ir)
            drive(pipuck, 0, sign * HARD_PIVOT_SPEED)
            print(f"[{now-start:5.1f}s] HARD-PIVOT ir_front={ir_front} sign={sign}")
            time.sleep(0.25)
            walk_state = "STRAIGHT"
            walk_state_until = time.time() + random.uniform(2.0, 4.5)
            continue

        if walk_state == "STRAIGHT":
            steer = 0
            if now >= walk_state_until:
                walk_state = "TURN"
                walk_turn_steer = random.choice((-1, 1)) * STEER_DELTA
                walk_state_until = now + random.uniform(0.3, 1.0)
        else:
            steer = walk_turn_steer
            if now >= walk_state_until:
                walk_state = "STRAIGHT"
                walk_state_until = now + random.uniform(2.0, 4.5)

        forward = FORWARD_SPEED
        override = None

        if ir_front > IR_SOFT:
            steer = ir_turn_sign(ir) * STEER_DELTA
            forward = FORWARD_SPEED // 2
            override = f"ir_soft({ir_front})"

        pose = get_pose()
        nearest = None
        if pose is not None:
            x, y, theta, _ = pose
            ahead_x = x + LOOKAHEAD * math.cos(theta)
            ahead_y = y + LOOKAHEAD * math.sin(theta)
            nearest = nearest_other(x, y)

            cx = 0.5 * (WORKSPACE_X[0] + WORKSPACE_X[1])
            cy = 0.5 * (WORKSPACE_Y[0] + WORKSPACE_Y[1])

            out_of_bounds = (
                x < WORKSPACE_X[0] or x > WORKSPACE_X[1]
                or y < WORKSPACE_Y[0] or y > WORKSPACE_Y[1]
            )
            zone_now = point_in_any_zone(x, y, zones, 0.0)

            if out_of_bounds or zone_now:
                diff = heading_to_center(x, y, theta)
                sign = 1 if diff > 0 else -1
                if abs(diff) > RECOVER_DIFF_TOLERANCE:
                    forward = 0
                    steer = sign * HARD_PIVOT_SPEED
                    override = f"RECOVER-pivot ({'oob' if out_of_bounds else 'Z!'} diff={math.degrees(diff):+.0f}°)"
                else:
                    forward = FORWARD_SPEED
                    steer = 0
                    override = f"RECOVER-drive ({'oob' if out_of_bounds else 'Z!'})"
                walk_state = "STRAIGHT"
                walk_state_until = now + random.uniform(2.0, 4.5)
            else:
                desired_x = 0.0
                desired_y = 0.0
                reasons = []

                wall_ahead = near_boundary(ahead_x, ahead_y)
                wall_close = near_boundary(x, y)
                if wall_close or wall_ahead:
                    dx, dy = cx - x, cy - y
                    norm = math.hypot(dx, dy) or 1.0
                    weight = 2.5 if wall_close else 1.0
                    desired_x += dx / norm * weight
                    desired_y += dy / norm * weight
                    reasons.append("wall" if wall_close else "wall_ahead")

                zone_ahead_check = point_in_any_zone(ahead_x, ahead_y, zones, ZONE_MARGIN)
                if zone_ahead_check:
                    dx, dy = cx - x, cy - y
                    norm = math.hypot(dx, dy) or 1.0
                    desired_x += dx / norm
                    desired_y += dy / norm
                    reasons.append("z")

                for rid, ox, oy in fresh_other_robots():
                    d_now = math.hypot(x - ox, y - oy)
                    d_ahead = math.hypot(ahead_x - ox, ahead_y - oy)
                    if d_now < OTHER_ROBOT_MARGIN or d_ahead < OTHER_ROBOT_LOOKAHEAD_MARGIN:
                        dx, dy = x - ox, y - oy
                        norm = math.hypot(dx, dy) or 1.0
                        weight = max(1.0, OTHER_ROBOT_MARGIN / max(d_now, 0.05))
                        desired_x += dx / norm * weight
                        desired_y += dy / norm * weight
                        reasons.append(f"#{rid}@{d_now:.2f}")

                if reasons:
                    if math.hypot(desired_x, desired_y) < 1e-3:
                        desired_x, desired_y = cx - x, cy - y
                    target = math.atan2(desired_y, desired_x)
                    diff = (target - theta + math.pi) % (2 * math.pi) - math.pi
                    sign = 1 if diff > 0 else -1
                    abs_diff = abs(diff)
                    if abs_diff > DIFF_PIVOT:
                        forward = 0
                        steer = sign * HARD_PIVOT_SPEED
                    elif abs_diff > DIFF_DEAD:
                        forward = FORWARD_SPEED
                        steer = int(sign * STEER_DELTA * (abs_diff / DIFF_PIVOT))
                    else:
                        forward = FORWARD_SPEED
                        steer = 0
                    override = "/".join(reasons[:3])
                    walk_state = "STRAIGHT"
                    walk_state_until = now + random.uniform(2.0, 4.5)

        if now - last_log >= LOG_INTERVAL:
            last_log = now
            if pose is not None:
                x, y, theta, _ = pose
                theta_deg = math.degrees(theta) % 360
                near_str = f"#{nearest[0]}@{nearest[3]:.2f}m" if nearest else "none"
                in_zone = point_in_any_zone(x, y, zones, 0.0)
                ahead_x = x + LOOKAHEAD * math.cos(theta)
                ahead_y = y + LOOKAHEAD * math.sin(theta)
                zone_ahead_now = point_in_any_zone(ahead_x, ahead_y, zones, ZONE_MARGIN)
                zone_flag = "Z!" if in_zone else ("z" if zone_ahead_now else "-")
                print(
                    f"[{now-start:5.1f}s] pos=({x:.2f},{y:.2f}) θ={theta_deg:5.1f}° "
                    f"state={walk_state} ir_f={ir_front} other={near_str} zone={zone_flag} "
                    f"override={override or '-'} fwd={forward} steer={steer:+d}"
                )
            else:
                print(
                    f"[{now-start:5.1f}s] pos=STALE ir_f={ir_front} state={walk_state} "
                    f"override={override or '-'} fwd={forward} steer={steer:+d}"
                )

        drive(pipuck, forward, steer)
        time.sleep(TICK)
finally:
    pipuck.epuck.set_motor_speeds(0, 0)
    client.loop_stop()
    client.disconnect()
