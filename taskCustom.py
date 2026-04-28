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
TOPIC = "robot_pos/all"
ZONES_URL = f"http://{BROKER}:3000/danger_zones.json"
ROBOT_ID = "32"

X_MIN, X_MAX = 0.1, 1.9
Y_MIN, Y_MAX = 0.1, 0.9
CENTER = (0.5 * (X_MIN + X_MAX), 0.5 * (Y_MIN + Y_MAX))

ZONE_MARGIN = 0.10
OTHER_MARGIN = 0.40
LOOKAHEAD = 0.30

FORWARD_SPEED = 750
WANDER_STEER = 320
MAX_STEER = 750
TURN_INTENSITY_FULL = math.radians(60)

TICK = 0.05
POSE_TIMEOUT = 1.5
OTHER_TIMEOUT = 3.0
LOG_INTERVAL = 0.5

STRAIGHT_MIN, STRAIGHT_MAX = 2.0, 4.5
TURN_MIN, TURN_MAX = 0.4, 1.0

RECOVER_DRIVE_TIME = 2.0


pose_lock = threading.Lock()
my_pose = {"x": None, "y": None, "theta": None, "t": 0.0}

others_lock = threading.Lock()
others = {}


def load_zones():
    try:
        with urllib.request.urlopen(ZONES_URL, timeout=3) as resp:
            data = json.loads(resp.read().decode())
        zs = data.get("zones", [])
        print(f"loaded {len(zs)} danger zone(s)")
        return zs
    except Exception as exc:
        print(f"could not load danger zones ({exc}); continuing without them")
        return []


def in_circle(x, y, z, margin):
    cx, cy = z["center"]["x"], z["center"]["y"]
    return math.hypot(x - cx, y - cy) <= z["radius"] + margin


def in_polygon(x, y, z, margin):
    pts = z["points"]
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
        seg2 = ex * ex + ey * ey
        t = 0.0 if seg2 == 0 else max(0.0, min(1.0, ((x - xj) * ex + (y - yj) * ey) / seg2))
        px, py = xj + t * ex, yj + t * ey
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < min_d2:
            min_d2 = d2
        j = i
    return min_d2 <= margin * margin


def hits_zone(x, y, zones, margin):
    for z in zones:
        kind = z.get("type")
        if kind == "circle" and in_circle(x, y, z, margin):
            return True
        if kind == "polygon" and in_polygon(x, y, z, margin):
            return True
    return False


def on_connect(client, userdata, flags, rc):
    print(f"connected mqtt rc={rc}")
    client.subscribe(TOPIC)


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        return
    if not isinstance(data, dict):
        return
    now = time.time()
    pending = {}
    for rid, entry in data.items():
        if not isinstance(entry, dict):
            continue
        pos = entry.get("position")
        if not (isinstance(pos, (list, tuple)) and len(pos) >= 2):
            continue
        try:
            x = float(pos[0])
            y = float(pos[1])
        except (TypeError, ValueError):
            continue
        if str(rid) == ROBOT_ID:
            try:
                theta = math.radians(float(entry.get("angle", 0.0)))
            except (TypeError, ValueError):
                continue
            with pose_lock:
                my_pose.update(x=x, y=y, theta=theta, t=now)
        else:
            pending[str(rid)] = (x, y, now)
    if pending:
        with others_lock:
            others.update(pending)


def get_pose():
    with pose_lock:
        if my_pose["x"] is None or time.time() - my_pose["t"] > POSE_TIMEOUT:
            return None
        return (my_pose["x"], my_pose["y"], my_pose["theta"])


def fresh_others():
    cutoff = time.time() - OTHER_TIMEOUT
    with others_lock:
        return [(rid, x, y) for rid, (x, y, t) in others.items() if t >= cutoff]


def out_of_bounds(x, y, margin=0.0):
    return (
        x < X_MIN + margin
        or x > X_MAX - margin
        or y < Y_MIN + margin
        or y > Y_MAX - margin
    )


def angle_diff(target, current):
    return (target - current + math.pi) % (2 * math.pi) - math.pi


def drive(pipuck, forward, steer):
    left = max(-1000, min(1000, int(forward + steer)))
    right = max(-1000, min(1000, int(forward - steer)))
    pipuck.epuck.set_motor_speeds(left, right)


def steer_toward(target, theta, max_steer=MAX_STEER):
    diff = angle_diff(target, theta)
    sign = 1 if diff >= 0 else -1
    intensity = min(1.0, abs(diff) / TURN_INTENSITY_FULL)
    return int(sign * max_steer * intensity)


def compute_avoid_vector(x, y, theta, zones):
    vx, vy = 0.0, 0.0
    reasons = []

    if hits_zone(x, y, zones, ZONE_MARGIN):
        dx, dy = CENTER[0] - x, CENTER[1] - y
        n = math.hypot(dx, dy) or 1.0
        vx += dx / n
        vy += dy / n
        reasons.append("zone")

    hx, hy = math.cos(theta), math.sin(theta)
    for rid, ox, oy in fresh_others():
        d = math.hypot(x - ox, y - oy)
        if d < OTHER_MARGIN:
            rx, ry = x - ox, y - oy
            n = d or 1.0
            rx, ry = rx / n, ry / n
            t1 = (-ry, rx)
            t2 = (ry, -rx)
            tx, ty = t1 if (t1[0] * hx + t1[1] * hy) >= (t2[0] * hx + t2[1] * hy) else t2
            repel_w = max(0.0, min(1.0, (OTHER_MARGIN - d) / OTHER_MARGIN))
            tang_w = 1.0 - repel_w
            weight = max(1.0, OTHER_MARGIN / max(d, 0.05))
            vx += weight * (repel_w * rx + tang_w * tx)
            vy += weight * (repel_w * ry + tang_w * ty)
            reasons.append(f"#{rid}@{d:.2f}")

    if not reasons:
        return None
    if math.hypot(vx, vy) < 1e-3:
        vx = CENTER[0] - x
        vy = CENTER[1] - y
    return (vx, vy, reasons)


zones = load_zones()

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()

pipuck = PiPuck(epuck_version=2)

start = time.time()
walk_state = "STRAIGHT"
walk_until = start + random.uniform(STRAIGHT_MIN, STRAIGHT_MAX)
turn_steer = WANDER_STEER
last_log = 0.0

recovery_until = 0.0

print("random walk started — Ctrl-C to stop")

try:
    while True:
        now = time.time()
        pose = get_pose()

        if pose is None:
            drive(pipuck, FORWARD_SPEED // 2, 0)
            if now - last_log >= LOG_INTERVAL:
                last_log = now
                print(f"[{now-start:6.1f}s] no fresh pose, crawling forward…")
            time.sleep(TICK)
            continue

        x, y, theta = pose
        ahead_x = x + LOOKAHEAD * math.cos(theta)
        ahead_y = y + LOOKAHEAD * math.sin(theta)

        forward = FORWARD_SPEED
        steer = 0
        mode = ""

        if now >= recovery_until and (
            out_of_bounds(x, y, 0.0) or out_of_bounds(ahead_x, ahead_y, 0.0)
        ):
            recovery_until = now + RECOVER_DRIVE_TIME

        if now < recovery_until:
            gx, gy = CENTER[0] - x, CENTER[1] - y
            gn = math.hypot(gx, gy) or 1.0
            gx, gy = gx / gn, gy / gn
            avoid = compute_avoid_vector(x, y, theta, zones)
            if avoid is not None:
                ax, ay, reasons = avoid
                tx, ty = gx + ax, gy + ay
                mode = f"RECOVER+AVOID({recovery_until - now:.1f}s,{','.join(reasons[:2])})"
            else:
                tx, ty = gx, gy
                mode = f"RECOVER({recovery_until - now:.1f}s)"
            target = math.atan2(ty, tx)
            steer = steer_toward(target, theta)
            walk_state = "STRAIGHT"
            walk_until = now + random.uniform(STRAIGHT_MIN, STRAIGHT_MAX)
        else:
            avoid = (
                compute_avoid_vector(x, y, theta, zones)
                or compute_avoid_vector(ahead_x, ahead_y, theta, zones)
            )

            if avoid is not None:
                vx, vy, reasons = avoid
                target = math.atan2(vy, vx)
                steer = steer_toward(target, theta)
                mode = "AVOID(" + "/".join(reasons[:3]) + ")"
                walk_state = "STRAIGHT"
                walk_until = now + random.uniform(STRAIGHT_MIN, STRAIGHT_MAX)
            else:
                if walk_state == "STRAIGHT":
                    steer = 0
                    if now >= walk_until:
                        walk_state = "TURN"
                        turn_steer = random.choice((-1, 1)) * WANDER_STEER
                        walk_until = now + random.uniform(TURN_MIN, TURN_MAX)
                else:
                    steer = turn_steer
                    if now >= walk_until:
                        walk_state = "STRAIGHT"
                        walk_until = now + random.uniform(STRAIGHT_MIN, STRAIGHT_MAX)
                mode = walk_state

        drive(pipuck, forward, steer)

        if now - last_log >= LOG_INTERVAL:
            last_log = now
            theta_deg = math.degrees(theta) % 360
            print(
                f"[{now-start:6.1f}s] pos=({x:.2f},{y:.2f}) θ={theta_deg:5.1f}° "
                f"mode={mode} fwd={forward} steer={steer:+d}"
            )

        time.sleep(TICK)

except KeyboardInterrupt:
    print("\nstopping (Ctrl-C)")
finally:
    pipuck.epuck.set_motor_speeds(0, 0)
    client.loop_stop()
    client.disconnect()
