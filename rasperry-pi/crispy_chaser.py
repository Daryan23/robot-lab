import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

MY_ID     = "26"
TARGET_ID = "32"

FOLLOW_DISTANCE = 0.20  # stop this far behind the target (metres)
ANGLE_START_DRIVING = 30
ANGLE_STOP_DRIVING  = 50
SPEED_BASE = 500
SPEED_TURN = 400

state = {
    "my_x": None, "my_y": None, "my_angle": None,
    "tgt_x": None, "tgt_y": None,
}


def on_connect(client, userdata, flags, rc):
    print("MQTT connected, rc=" + str(rc))
    client.subscribe("robot_pos/all")


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        me = data.get(MY_ID)
        tgt = data.get(TARGET_ID)
        if me:
            state["my_x"]     = float(me["position"][0])
            state["my_y"]     = float(me["position"][1])
            state["my_angle"] = float(me["angle"])
        if tgt:
            state["tgt_x"] = float(tgt["position"][0])
            state["tgt_y"] = float(tgt["position"][1])
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        pass


def angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def main():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(Broker, Port, 60)
    mqtt_client.loop_start()

    print("Waiting for own position fix...")
    while state["my_x"] is None:
        time.sleep(0.1)
    print(f"Got position: ({state['my_x']:.2f}, {state['my_y']:.2f})")
    print(f"Searching for robot {TARGET_ID}...")

    pipuck = PiPuck(epuck_version=2)

    try:
        while True:
            mx, my, ma = state["my_x"], state["my_y"], state["my_angle"]
            tx, ty     = state["tgt_x"], state["tgt_y"]

            if None in (mx, my, ma):
                pipuck.epuck.set_motor_speeds(0, 0)
                time.sleep(0.05)
                continue

            # Target not visible — spin to search
            if tx is None or ty is None:
                print(f"Robot {TARGET_ID} not visible, searching...")
                pipuck.epuck.set_motor_speeds(SPEED_TURN, -SPEED_TURN)
                time.sleep(0.1)
                continue

            dx = tx - mx
            dy = ty - my
            dist = math.hypot(dx, dy)

            print(f"dist={dist:.2f} target=({tx:.2f},{ty:.2f}) me=({mx:.2f},{my:.2f})")

            if dist <= FOLLOW_DISTANCE:
                pipuck.epuck.set_motor_speeds(0, 0)
                time.sleep(0.05)
                continue

            # Compass heading toward target
            hdg = math.degrees(math.atan2(dx, dy)) % 360
            err = angle_diff(hdg, ma)

            if abs(err) > ANGLE_STOP_DRIVING:
                if err > 0:
                    pipuck.epuck.set_motor_speeds(SPEED_TURN, -SPEED_TURN)
                else:
                    pipuck.epuck.set_motor_speeds(-SPEED_TURN, SPEED_TURN)
            elif abs(err) < ANGLE_START_DRIVING:
                pipuck.epuck.set_motor_speeds(SPEED_BASE, SPEED_BASE)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        pipuck.epuck.set_motor_speeds(0, 0)
        mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
