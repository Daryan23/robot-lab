import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

ROBOT_ID = "26"

MARGIN = 0.15

WAYPOINTS = [
    (MARGIN,        MARGIN),
    (2.0 - MARGIN,  MARGIN),
    (2.0 - MARGIN,  1.0 - MARGIN),
    (MARGIN,        1.0 - MARGIN),
]

WAYPOINT_THRESHOLD = 0.08
SPEED_BASE = 500
SPEED_TURN = 400
ANGLE_TOL  = 25  # degrees

# Camera angle convention:
#   "math"    → 0° = right (+x), counter-clockwise positive  (atan2(dy, dx))
#   "compass" → 0° = up   (+y), clockwise positive           (atan2(dx, dy))
ANGLE_CONVENTION = "compass"

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


def target_heading(dx, dy):
    if ANGLE_CONVENTION == "compass":
        return math.degrees(math.atan2(dx, dy)) % 360
    else:
        return math.degrees(math.atan2(dy, dx)) % 360


def angle_diff(target, current):
    """Signed shortest difference. Positive = turn CW (compass convention)."""
    return (target - current + 180) % 360 - 180


def drive_to(pipuck, wx, wy, label=""):
    print(f"→ waypoint {label}({wx:.2f}, {wy:.2f})")
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
            pipuck.epuck.set_motor_speeds(0, 0)
            return

        hdg = target_heading(dx, dy)
        err = angle_diff(hdg, angle)

        if abs(err) > ANGLE_TOL:
            # Stop and turn in place until aligned
            if err > 0:
                pipuck.epuck.set_motor_speeds(SPEED_TURN, -SPEED_TURN)
            else:
                pipuck.epuck.set_motor_speeds(-SPEED_TURN, SPEED_TURN)
        else:
            # Aligned — drive straight, no correction
            pipuck.epuck.set_motor_speeds(SPEED_BASE, SPEED_BASE)

        print(f"  pos=({x:.2f},{y:.2f}) dist={dist:.2f} hdg={hdg:.0f}° robot={angle:.0f}° err={err:.1f}°")
        time.sleep(0.05)


def nearest_waypoint_index():
    x, y = robot_state["x"], robot_state["y"]
    return min(range(len(WAYPOINTS)), key=lambda i: math.hypot(WAYPOINTS[i][0] - x, WAYPOINTS[i][1] - y))


def main():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(Broker, Port, 60)
    mqtt_client.loop_start()

    print("Waiting for position fix...")
    while robot_state["x"] is None:
        time.sleep(0.1)
    print(f"Position: ({robot_state['x']:.2f}, {robot_state['y']:.2f})")

    pipuck = PiPuck(epuck_version=2)

    try:
        start_idx = nearest_waypoint_index()
        print(f"Nearest corner: {WAYPOINTS[start_idx]}, starting perimeter walk from there.")

        idx = start_idx
        while True:
            wx, wy = WAYPOINTS[idx % len(WAYPOINTS)]
            drive_to(pipuck, wx, wy, label=f"#{idx % len(WAYPOINTS)} ")
            idx += 1

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        pipuck.epuck.set_motor_speeds(0, 0)
        mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
