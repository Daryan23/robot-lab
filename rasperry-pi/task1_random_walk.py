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

WORKSPACE_X = (0.0, 2.0)
WORKSPACE_Y = (0.0, 1.0)
BOUNDARY_MARGIN = 0.15
ZONE_MARGIN = 0.08
LOOKAHEAD = 0.20

FORWARD_SPEED = 600
STEER_DELTA = 350
HARD_PIVOT_SPEED = 450
IR_SOFT = 600
IR_HARD = 1500
POSE_TIMEOUT = 5.0
TICK = 0.05
RUN_DURATION = 60.0

pose_lock = threading.Lock()
latest_pose = {"x": None, "y": None, "theta": None, "in_danger": False, "t": 0.0}


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
    entry = data.get(str(ROBOT_ID)) or data.get(ROBOT_ID)
    if not isinstance(entry, dict):
        return
    pos = entry.get("position")
    if not (isinstance(pos, (list, tuple)) and len(pos) >= 2):
        return
    try:
        x = float(pos[0])
        y = float(pos[1])
        theta = math.radians(float(entry.get("angle", 0.0)))
    except (TypeError, ValueError):
        return
    in_danger = bool(entry.get("in_danger", False))
    with pose_lock:
        latest_pose.update(x=x, y=y, theta=theta, in_danger=in_danger, t=time.time())


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
random_steer = 0
random_steer_until = start

try:
    while time.time() - start < RUN_DURATION:
        now = time.time()
        ir = pipuck.epuck.ir_reflected
        ir_front = ir_front_max(ir)

        if ir_front > IR_HARD:
            sign = ir_turn_sign(ir)
            drive(pipuck, 0, sign * HARD_PIVOT_SPEED)
            time.sleep(0.25)
            random_steer = 0
            random_steer_until = time.time()
            continue

        if now >= random_steer_until:
            random_steer = random.choice((-1, 0, 0, 1)) * STEER_DELTA // 2
            random_steer_until = now + random.uniform(0.8, 2.5)

        steer = random_steer
        forward = FORWARD_SPEED

        if ir_front > IR_SOFT:
            steer = ir_turn_sign(ir) * STEER_DELTA
            forward = FORWARD_SPEED // 2

        pose = get_pose()
        if pose is not None:
            x, y, theta, in_danger = pose
            override = False
            if in_danger:
                diff = heading_to_center(x, y, theta)
                steer = (1 if diff > 0 else -1) * STEER_DELTA
                forward = FORWARD_SPEED // 2
                override = True
            else:
                ahead_x = x + LOOKAHEAD * math.cos(theta)
                ahead_y = y + LOOKAHEAD * math.sin(theta)
                if near_boundary(x, y) or point_in_any_zone(ahead_x, ahead_y, zones, ZONE_MARGIN):
                    diff = heading_to_center(x, y, theta)
                    steer = (1 if diff > 0 else -1) * STEER_DELTA
                    override = True
            if override:
                random_steer_until = now + random.uniform(0.8, 2.5)

        drive(pipuck, forward, steer)
        time.sleep(TICK)
finally:
    pipuck.epuck.set_motor_speeds(0, 0)
    client.loop_stop()
    client.disconnect()
