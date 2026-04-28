import paho.mqtt.client as mqtt
import json
import math
import random
import threading
import time
from pipuck.pipuck import PiPuck

BROKER = "192.168.178.43"
PORT = 1883
ROBOT_ID = 32

WORKSPACE_X = (0.0, 2.0)
WORKSPACE_Y = (0.0, 1.0)
BOUNDARY_MARGIN = 0.15

FORWARD_SPEED = 600
TURN_SPEED = 400
IR_THRESHOLD = 200
POSE_TIMEOUT = 3.0
TICK = 0.05
RUN_DURATION = 60.0

pose_lock = threading.Lock()
latest_pose = {"x": None, "y": None, "theta": None, "t": 0.0}


def extract_pose(payload, robot_id):
    if isinstance(payload, list):
        for entry in payload:
            if isinstance(entry, dict) and entry.get("id") == robot_id:
                return entry
        return None
    if isinstance(payload, dict):
        if payload.get("id") == robot_id:
            return payload
        keyed = payload.get(str(robot_id)) or payload.get(robot_id)
        if isinstance(keyed, dict):
            return keyed
    return None


def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("robot_pos/all")


_debug_msgs_logged = 0


def on_message(client, userdata, msg):
    global _debug_msgs_logged
    try:
        data = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        print(f"invalid json: {msg.payload}")
        return
    if _debug_msgs_logged < 3:
        print(f"[debug] raw payload: {data!r}")
        _debug_msgs_logged += 1
    pose = extract_pose(data, ROBOT_ID)
    if pose is None:
        if _debug_msgs_logged <= 3:
            print(f"[debug] extract_pose returned None for ID {ROBOT_ID}")
        return
    try:
        x = float(pose["x"])
        y = float(pose["y"])
        theta = float(pose.get("theta", pose.get("yaw", 0.0)))
    except (KeyError, TypeError, ValueError):
        return
    with pose_lock:
        latest_pose.update(x=x, y=y, theta=theta, t=time.time())


def get_pose():
    with pose_lock:
        if latest_pose["x"] is None:
            return None
        if time.time() - latest_pose["t"] > POSE_TIMEOUT:
            return None
        return latest_pose["x"], latest_pose["y"], latest_pose["theta"]


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
    diff = (target - theta + math.pi) % (2 * math.pi) - math.pi
    return diff


def front_obstacle(ir):
    # Sensors 0 and 7 face forward; 1 and 6 are front-diagonals.
    return ir[0] > IR_THRESHOLD or ir[7] > IR_THRESHOLD


def turn_direction_for_obstacle(ir):
    return -1 if (ir[0] + ir[1]) > (ir[7] + ir[6]) else 1


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()

pipuck = PiPuck(epuck_version=2)

start = time.time()
state = "FORWARD"
state_until = start + random.uniform(1.5, 3.0)
turn_sign = 1

try:
    while time.time() - start < RUN_DURATION:
        now = time.time()
        ir = pipuck.epuck.ir_reflected

        if front_obstacle(ir):
            sign = turn_direction_for_obstacle(ir)
            pipuck.epuck.set_motor_speeds(sign * TURN_SPEED, -sign * TURN_SPEED)
            time.sleep(random.uniform(0.4, 0.9))
            state = "FORWARD"
            state_until = time.time() + random.uniform(1.5, 3.0)
            continue

        pose = get_pose()
        if pose is None:
            pipuck.epuck.set_motor_speeds(0, 0)
            print("no pose — holding")
            time.sleep(0.2)
            continue

        x, y, theta = pose

        if state == "FORWARD":
            if near_boundary(x, y):
                diff = heading_to_center(x, y, theta)
                turn_sign = 1 if diff > 0 else -1
                state = "TURN"
                state_until = now + min(2.0, abs(diff) / 1.5 + 0.3)
            elif now >= state_until:
                state = "TURN"
                turn_sign = random.choice((-1, 1))
                state_until = now + random.uniform(0.3, 1.2)

        if state == "TURN":
            pipuck.epuck.set_motor_speeds(turn_sign * TURN_SPEED, -turn_sign * TURN_SPEED)
            if now >= state_until:
                state = "FORWARD"
                state_until = now + random.uniform(1.5, 3.0)
        else:
            pipuck.epuck.set_motor_speeds(FORWARD_SPEED, FORWARD_SPEED)

        time.sleep(TICK)
finally:
    pipuck.epuck.set_motor_speeds(0, 0)
    client.loop_stop()
    client.disconnect()
