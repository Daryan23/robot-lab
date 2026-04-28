import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

ROBOT_ID = "26"

MARGIN = 0.15  # distance from wall to follow

# Waypoints: corners of the field inset by MARGIN, traversed clockwise
# Field: x 0-2m, y 0-1m
WAYPOINTS = [
    (MARGIN,        MARGIN),        # bottom-left
    (2.0 - MARGIN,  MARGIN),        # bottom-right
    (2.0 - MARGIN,  1.0 - MARGIN),  # top-right
    (MARGIN,        1.0 - MARGIN),  # top-left
]

WAYPOINT_THRESHOLD = 0.08  # metres — how close counts as "reached"

SPEED_BASE   = 500
SPEED_TURN   = 400
ANGLE_TOL    = 8   # degrees — start driving straight within this heading error

robot_state = {"x": None, "y": None, "angle": None}


def on_connect(client, userdata, flags, rc):
    print("MQTT connected, rc=" + str(rc))
    client.subscribe("robot_pos/all")


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        entry = data.get(ROBOT_ID)
        if entry:
            robot_state["x"]     = float(entry["position"][0])
            robot_state["y"]     = float(entry["position"][1])
            robot_state["angle"] = float(entry["angle"])
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        pass


def angle_diff(target, current):
    """Shortest signed difference in degrees: positive = turn left."""
    diff = (target - current + 180) % 360 - 180
    return diff


def drive_to_waypoint(pipuck, wx, wy):
    """Block until the robot reaches (wx, wy). Returns False if position lost."""
    print(f"Heading to waypoint ({wx:.2f}, {wy:.2f})")
    while True:
        x, y, angle = robot_state["x"], robot_state["y"], robot_state["angle"]
        if x is None:
            time.sleep(0.05)
            continue

        dx = wx - x
        dy = wy - y
        dist = math.hypot(dx, dy)

        if dist < WAYPOINT_THRESHOLD:
            print(f"  reached ({x:.2f}, {y:.2f})")
            return True

        # Target heading: camera uses standard math angle (0° = right, CCW positive)
        # Adjust if your robot's angle convention differs
        target_angle = math.degrees(math.atan2(dy, dx))
        err = angle_diff(target_angle, angle)

        if abs(err) > ANGLE_TOL:
            # Turn toward target
            if err > 0:
                pipuck.epuck.set_motor_speeds(-SPEED_TURN, SPEED_TURN)
            else:
                pipuck.epuck.set_motor_speeds(SPEED_TURN, -SPEED_TURN)
        else:
            # Drive forward, slight correction
            correction = int(err * 3)
            pipuck.epuck.set_motor_speeds(SPEED_BASE - correction, SPEED_BASE + correction)

        print(f"  pos=({x:.2f},{y:.2f}) dist={dist:.2f} err={err:.1f}°")
        time.sleep(0.05)


def main():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(Broker, Port, 60)
    mqtt_client.loop_start()

    print("Waiting for first position fix...")
    while robot_state["x"] is None:
        time.sleep(0.1)
    print(f"Got position: ({robot_state['x']:.2f}, {robot_state['y']:.2f})")

    pipuck = PiPuck(epuck_version=2)

    try:
        idx = 0
        while True:
            wx, wy = WAYPOINTS[idx % len(WAYPOINTS)]
            drive_to_waypoint(pipuck, wx, wy)
            idx += 1
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        pipuck.epuck.set_motor_speeds(0, 0)
        mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
