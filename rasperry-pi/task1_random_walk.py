import paho.mqtt.client as mqtt
import json
import time
import random
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

# Workspace boundaries: 2 m wide (x) × 1 m deep (y), centred at origin
X_MIN, X_MAX = 0, 2.0
Y_MIN, Y_MAX = 0, 1.0
BOUNDARY_MARGIN = 0.15  # start turning this far from the edge

# Motor speed constants
SPEED_FORWARD = 600
SPEED_TURN = 500

# How often to pick a new random direction (seconds)
WALK_INTERVAL_MIN = 1.5
WALK_INTERVAL_MAX = 4.0

RUN_DURATION = 120  # total seconds to run

ROBOT_ID = "26"

robot_pos = {"x": None, "y": None}  # updated by MQTT


def on_connect(client, userdata, flags, rc):
    print("MQTT connected, rc=" + str(rc))
    client.subscribe("robot_pos/all")


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        entry = data.get(ROBOT_ID)
        if entry:
            robot_pos["x"] = float(entry["position"][0])
            robot_pos["y"] = float(entry["position"][1])
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        pass


def near_boundary():
    x, y = robot_pos["x"], robot_pos["y"]
    if x is None or y is None:
        return False
    return (
        x < X_MIN + BOUNDARY_MARGIN
        or x > X_MAX - BOUNDARY_MARGIN
        or y < Y_MIN + BOUNDARY_MARGIN
        or y > Y_MAX - BOUNDARY_MARGIN
    )


def random_walk(pipuck):
    end_time = time.time() + RUN_DURATION
    next_change = time.time()

    # State: "forward" or "turn"
    state = "forward"
    turn_end = 0

    while time.time() < end_time:
        now = time.time()

        if near_boundary():
            # Spin in place to point away from the wall
            pipuck.epuck.set_motor_speeds(-SPEED_TURN, SPEED_TURN)
            time.sleep(0.4 + random.uniform(0.2, 0.6))
            next_change = now + random.uniform(WALK_INTERVAL_MIN, WALK_INTERVAL_MAX)
            state = "forward"
            continue

        if state == "turn" and now < turn_end:
            pass  # keep turning
        elif now >= next_change:
            # Pick a new behaviour
            choice = random.random()
            if choice < 0.6:
                # Drive straight
                pipuck.epuck.set_motor_speeds(SPEED_FORWARD, SPEED_FORWARD)
                state = "forward"
            elif choice < 0.8:
                # Turn left
                pipuck.epuck.set_motor_speeds(-SPEED_TURN, SPEED_TURN)
                turn_end = now + random.uniform(0.3, 0.8)
                state = "turn"
            else:
                # Turn right
                pipuck.epuck.set_motor_speeds(SPEED_TURN, -SPEED_TURN)
                turn_end = now + random.uniform(0.3, 0.8)
                state = "turn"

            next_change = now + random.uniform(WALK_INTERVAL_MIN, WALK_INTERVAL_MAX)

        pos_str = (
            f"x={robot_pos['x']:.2f} y={robot_pos['y']:.2f}"
            if robot_pos["x"] is not None
            else "position unknown"
        )
        print(f"[{now:.1f}] state={state} {pos_str}")
        time.sleep(0.1)


def main():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(Broker, Port, 60)
    mqtt_client.loop_start()

    pipuck = PiPuck(epuck_version=2)

    try:
        random_walk(pipuck)
    finally:
        pipuck.epuck.set_motor_speeds(0, 0)
        mqtt_client.loop_stop()
        print("Done.")


if __name__ == "__main__":
    main()
