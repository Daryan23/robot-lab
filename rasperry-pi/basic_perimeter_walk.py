import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

ROBOT_ID = "26"

WAYPOINTS = [
    (0.1, 0.1),
    (0.1, 1.0),
    (1.9, 1.0),
    (1.9, 0.1),
]

WAYPOINT_THRESHOLD = 0.08
SPEED_BASE = 500
SPEED_TURN = 400
ANGLE_TURN_THRESHOLD = 30

# 0° = +y (north), CW positive — matches camera tracker output.
ANGLE_CONVENTION = "compass"

robots_state = {}


def on_connect(client, userdata, flags, rc):
    print("MQTT connected, rc=" + str(rc))
    client.subscribe("robot_pos/all")


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        return
    now = time.time()
    for rid, entry in data.items():
        try:
            robots_state[rid] = {
                "x":         float(entry["position"][0]),
                "y":         float(entry["position"][1]),
                "angle":     float(entry["angle"]),
                "last_seen": now,
            }
        except (KeyError, ValueError, TypeError):
            continue


def heading_to(dx, dy):
    if ANGLE_CONVENTION == "compass":
        return math.degrees(math.atan2(dx, dy)) % 360
    return math.degrees(math.atan2(dy, dx)) % 360


def angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def main():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(Broker, Port, 60)
    mqtt_client.loop_start()

    print("Waiting for position fix...")
    while ROBOT_ID not in robots_state:
        time.sleep(0.1)
    st = robots_state[ROBOT_ID]
    print(f"Position: ({st['x']:.2f}, {st['y']:.2f})")

    pipuck = PiPuck(epuck_version=2)
    target_idx = 0

    try:
        while True:
            st = robots_state.get(ROBOT_ID)
            if st is None:
                time.sleep(0.05)
                continue
            x, y, angle = st["x"], st["y"], st["angle"]

            tx, ty = WAYPOINTS[target_idx % len(WAYPOINTS)]
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)

            if dist < WAYPOINT_THRESHOLD:
                print(f"  reached waypoint #{target_idx % len(WAYPOINTS)} ({x:.2f},{y:.2f})")
                target_idx += 1
                continue

            hdg = heading_to(dx, dy)
            err = angle_diff(hdg, angle)

            if abs(err) > ANGLE_TURN_THRESHOLD:
                if err > 0:
                    left, right = SPEED_TURN, -SPEED_TURN
                else:
                    left, right = -SPEED_TURN, SPEED_TURN
            else:
                left, right = SPEED_BASE, SPEED_BASE
            pipuck.epuck.set_motor_speeds(left, right)

            print(f"  wp #{target_idx % len(WAYPOINTS)} pos=({x:.2f},{y:.2f}) "
                  f"tgt=({tx:.2f},{ty:.2f}) dist={dist:.2f} "
                  f"hdg={hdg:.0f}° robot={angle:.0f}° err={err:+.0f}° "
                  f"motors=({left},{right})")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        pipuck.epuck.set_motor_speeds(0, 0)
        mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
