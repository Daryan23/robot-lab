import paho.mqtt.client as mqtt
import json
import time
import math
import random
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

MY_ID = "26"

CIRCLE_RADIUS = 0.05   # how far from the target to orbit (metres)
APPROACH_TOL  = 0.05   # acceptable radius error before switching to orbit
DIRECTION     = 1      # +1 = counter-clockwise, -1 = clockwise

SPEED_BASE = 450       # forward speed when going around the circle
STEER_GAIN = 8         # proportional gain on heading error (motor units per degree)
STALE_S    = 1.5

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
                "x":     float(entry["position"][0]),
                "y":     float(entry["position"][1]),
                "angle": float(entry["angle"]),
                "ts":    now,
            }
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        pass


def angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def random_other():
    """Return (id, x, y) of a random fresh robot that isn't us, or None."""
    now = time.time()
    candidates = [
        (rid, st["x"], st["y"])
        for rid, st in robots_state.items()
        if rid != MY_ID and now - st["ts"] <= STALE_S
    ]
    return random.choice(candidates) if candidates else None


def main():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(Broker, Port, 60)
    mqtt_client.loop_start()

    print("Waiting for own position fix...")
    while MY_ID not in robots_state:
        time.sleep(0.1)
    me = robots_state[MY_ID]
    print(f"Got position: ({me['x']:.2f}, {me['y']:.2f})")

    # Lock target ONCE: pick a random other robot, remember its coordinates
    print("Picking a random other robot...")
    target = None
    while target is None:
        target = random_other()
        if target is None:
            time.sleep(0.1)
    tid, tx, ty = target
    print(f"Locked target {tid} at fixed coords ({tx:.2f}, {ty:.2f}). Orbiting forever.")

    pipuck = PiPuck(epuck_version=2)

    try:
        while True:
            me = robots_state.get(MY_ID)
            if me is None:
                time.sleep(0.05)
                continue
            mx, my, ma = me["x"], me["y"], me["angle"]

            # Vector from target to me
            rx, ry = mx - tx, my - ty
            r_mag  = math.hypot(rx, ry) or 1e-6
            rxn, ryn = rx / r_mag, ry / r_mag

            # Tangent direction (perpendicular to radial), oriented by DIRECTION
            # In compass-style coords (x=right, y=up), CCW tangent of radial (rx,ry)
            # is (-ry, rx). Flip sign for CW.
            tnx = -ryn * DIRECTION
            tny =  rxn * DIRECTION

            # Radial correction: pull inward if too far, push outward if too close
            radial_err = r_mag - CIRCLE_RADIUS  # positive = too far, drive inward
            radial_gain = max(-1.0, min(1.0, radial_err / 0.20))
            # Inward unit vector (toward target) is (-rxn, -ryn)
            cx = -rxn * radial_gain
            cy = -ryn * radial_gain

            # Aim direction = tangent blended with radial correction
            ax = tnx + cx
            ay = tny + cy
            aim_mag = math.hypot(ax, ay) or 1e-6
            ax /= aim_mag
            ay /= aim_mag

            hdg = math.degrees(math.atan2(ax, ay)) % 360  # compass: atan2(dx, dy)
            err = angle_diff(hdg, ma)

            mode = "orbit" if abs(radial_err) < APPROACH_TOL else ("approach" if radial_err > 0 else "back-off")
            print(f"center=({tx:.2f},{ty:.2f}) dist={r_mag:.2f} radial_err={radial_err:+.2f} mode={mode} err={err:+.0f}°")

            # Smooth proportional steering: positive err = need to turn CW
            # (right wheel slower). Always drive forward, just bias the wheels.
            steer = STEER_GAIN * err
            left  = SPEED_BASE + steer
            right = SPEED_BASE - steer
            # Clamp to reasonable motor range
            left  = max(-SPEED_BASE, min(SPEED_BASE * 2, left))
            right = max(-SPEED_BASE, min(SPEED_BASE * 2, right))
            pipuck.epuck.set_motor_speeds(int(left), int(right))

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        pipuck.epuck.set_motor_speeds(0, 0)
        mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
