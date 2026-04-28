import paho.mqtt.client as mqtt
import json
import time
import math
import threading
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

MY_ID     = "26"
TARGET_ID = "32"

SPEED_BASE          = 600
SPEED_TURN          = 450
ANGLE_START_DRIVING = 30
ANGLE_STOP_DRIVING  = 50

OBSTACLE_RADIUS  = 0.25   # only check robots within this range ahead
OBSTACLE_LATERAL = 0.12   # robot counts as blocker if within 12 cm of our line
EVADE_DURATION   = 0.6    # seconds to steer around a blocker

BLINK_DIST = 0.30         # start blinking red below this distance

robots_state = {}


def on_connect(client, userdata, flags, rc):
    print("MQTT connected, rc=" + str(rc))
    client.subscribe("robot_pos/all")


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        now = time.time()
        for rid, entry in data.items():
            robots_state[rid] = {
                "x":         float(entry["position"][0]),
                "y":         float(entry["position"][1]),
                "angle":     float(entry["angle"]),
                "ts":        now,
            }
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        pass


def angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def obstacle_in_path(mx, my, tx, ty):
    """Return (ox, oy) of the closest non-target robot blocking our path, or None."""
    fdx, fdy = tx - mx, ty - my
    L = math.hypot(fdx, fdy)
    if L < 1e-6:
        return None
    fx, fy = fdx / L, fdy / L

    now = time.time()
    closest, closest_d = None, float("inf")
    for rid, st in robots_state.items():
        if rid in (MY_ID, TARGET_ID):
            continue
        if now - st["ts"] > 1.5:
            continue
        rx, ry = st["x"] - mx, st["y"] - my
        f = rx * fx + ry * fy
        if f <= 0.02 or f > OBSTACLE_RADIUS:
            continue
        lat = math.hypot(rx - f * fx, ry - f * fy)
        if lat > OBSTACLE_LATERAL:
            continue
        d = math.hypot(rx, ry)
        if d < closest_d:
            closest_d = d
            closest = (st["x"], st["y"])
    return closest


# LED blink state
_blink_active = False
_blink_lock   = threading.Lock()


def set_leds_red(pipuck, on):
    try:
        pipuck.epuck.set_leds(0xFF if on else 0x00)
    except Exception:
        pass


def blink_loop(pipuck):
    state = False
    while True:
        with _blink_lock:
            active = _blink_active
        if active:
            state = not state
            set_leds_red(pipuck, state)
            time.sleep(0.15)
        else:
            set_leds_red(pipuck, False)
            state = False
            time.sleep(0.05)


def main():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(Broker, Port, 60)
    mqtt_client.loop_start()

    print("Waiting for own position fix...")
    while MY_ID not in robots_state:
        time.sleep(0.1)
    st = robots_state[MY_ID]
    print(f"Got position: ({st['x']:.2f}, {st['y']:.2f})")
    print(f"Hunting robot {TARGET_ID}...")

    pipuck = PiPuck(epuck_version=2)

    blink_thread = threading.Thread(target=blink_loop, args=(pipuck,), daemon=True)
    blink_thread.start()

    evade_until = 0
    evade_dir   = 1  # +1 = left, -1 = right

    try:
        global _blink_active
        while True:
            me  = robots_state.get(MY_ID)
            tgt = robots_state.get(TARGET_ID)

            if me is None:
                time.sleep(0.05)
                continue

            mx, my, ma = me["x"], me["y"], me["angle"]

            # No target visible — spin to search
            if tgt is None:
                print(f"Robot {TARGET_ID} not visible, searching...")
                with _blink_lock:
                    _blink_active = False
                pipuck.epuck.set_motor_speeds(SPEED_TURN, -SPEED_TURN)
                time.sleep(0.1)
                continue

            tx, ty = tgt["x"], tgt["y"]
            dist = math.hypot(tx - mx, ty - my)

            print(f"dist={dist:.2f} me=({mx:.2f},{my:.2f}) target=({tx:.2f},{ty:.2f})")

            # Blink red when close
            with _blink_lock:
                _blink_active = dist < BLINK_DIST

            now = time.time()
            if now < evade_until:
                # Actively evading an obstacle — steer around it
                pipuck.epuck.set_motor_speeds(
                    SPEED_BASE if evade_dir > 0 else SPEED_TURN,
                    SPEED_BASE if evade_dir < 0 else SPEED_TURN,
                )
                time.sleep(0.05)
                continue

            # Check for obstacle in path toward target
            obs = obstacle_in_path(mx, my, tx, ty)
            if obs is not None:
                ox, oy = obs
                # Steer around: pick side away from obstacle
                fdx, fdy = tx - mx, ty - my
                L = math.hypot(fdx, fdy)
                fx, fy = fdx / L, fdy / L
                # Perpendicular: if obstacle is to the left of forward, evade right
                cross = fx * (oy - my) - fy * (ox - mx)
                evade_dir   = -1 if cross > 0 else 1
                evade_until = now + EVADE_DURATION
                print(f"Obstacle at ({ox:.2f},{oy:.2f}), evading {'left' if evade_dir>0 else 'right'}")
                continue

            # Drive straight at target
            hdg = math.degrees(math.atan2(tx - mx, ty - my)) % 360
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
        with _blink_lock:
            _blink_active = False
        time.sleep(0.2)
        pipuck.epuck.set_motor_speeds(0, 0)
        mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
