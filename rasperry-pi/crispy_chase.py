import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

MY_ID     = "26"
TARGET_ID = "32"

# Behaviour tuning ----------------------------------------------------------
FOLLOW_DISTANCE      = 0.20    # stop this far from the target (metres)
RESUME_DISTANCE      = 0.28    # hysteresis: only restart chase past this
ANGLE_TURN_THRESHOLD = 25      # spin in place above this; drive otherwise
SPEED_BASE           = 500
SPEED_TURN           = 400

# Obstacle avoidance: treat any other robot (not me, not target) ahead of us
# and close to our line of motion as a blocker we should steer around.
OBSTACLE_RADIUS  = 0.30   # only react to obstacles closer than this
OBSTACLE_LATERAL = 0.10   # only react if within this distance of our path
OBSTACLE_STALE_S = 1.5    # ignore positions older than this
DEFLECT_OFFSET   = 0.22   # how far to step sideways past the blocker

# Angle convention used by the camera tracker: 0° = +y (north), CW positive.
ANGLE_CONVENTION = "compass"

robots_state = {}  # rid -> {"x", "y", "angle", "last_seen"}


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


def angle_diff(target, current):
    """Signed shortest difference. Positive = turn CW (compass convention)."""
    return (target - current + 180) % 360 - 180


def heading_to(dx, dy):
    if ANGLE_CONVENTION == "compass":
        return math.degrees(math.atan2(dx, dy)) % 360
    return math.degrees(math.atan2(dy, dx)) % 360


def fresh(st):
    return st is not None and (time.time() - st["last_seen"]) <= OBSTACLE_STALE_S


def closest_blocker(my_x, my_y, tx, ty):
    """Closest non-me, non-target robot inside the forward corridor.

    Returns (id, x, y, lat_sgn) where lat_sgn > 0 means the blocker is on
    our left (compass), < 0 means on our right.
    """
    fdx, fdy = tx - my_x, ty - my_y
    L = math.hypot(fdx, fdy)
    if L < 1e-6:
        return None
    fx, fy = fdx / L, fdy / L

    closest = None
    closest_d = float("inf")
    now = time.time()
    for rid, st in robots_state.items():
        if rid == MY_ID or rid == TARGET_ID:
            continue
        if now - st.get("last_seen", 0) > OBSTACLE_STALE_S:
            continue
        ox, oy = st["x"], st["y"]
        rx, ry = ox - my_x, oy - my_y
        forward = rx * fx + ry * fy
        if forward <= 0.02 or forward > min(OBSTACLE_RADIUS, L - 0.02):
            continue
        # Signed lateral in compass frame: +ve = blocker on our left.
        lat_sgn = -rx * fy + ry * fx
        if abs(lat_sgn) > OBSTACLE_LATERAL:
            continue
        d = math.hypot(rx, ry)
        if d < closest_d:
            closest_d = d
            closest = (rid, ox, oy, lat_sgn)
    return closest


def aim_with_avoidance(my_x, my_y, tx, ty):
    """Pick an aim point — straight at target, or alongside a blocker."""
    blocker = closest_blocker(my_x, my_y, tx, ty)
    if blocker is None:
        return tx, ty, None
    rid, ox, oy, lat_sgn = blocker

    fdx, fdy = tx - my_x, ty - my_y
    L = math.hypot(fdx, fdy)
    fx, fy = fdx / L, fdy / L
    forward = (ox - my_x) * fx + (oy - my_y) * fy

    # Deflect away from whichever side the blocker is on. In compass frame
    # (-fy, fx) is left of forward, (fy, -fx) is right.
    if lat_sgn >= 0:           # blocker on left → step right
        px, py = fy, -fx
    else:                      # blocker on right → step left
        px, py = -fy, fx

    aim_x = my_x + (forward + 0.05) * fx + DEFLECT_OFFSET * px
    aim_y = my_y + (forward + 0.05) * fy + DEFLECT_OFFSET * py
    return aim_x, aim_y, rid


def set_lights(pipuck, on):
    """Pi-puck RGB ring: red when locked on target, off while chasing."""
    try:
        pipuck.set_leds_colour("red" if on else "off")
    except Exception as e:
        print(f"  (LED control unavailable: {e})")


def main():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(Broker, Port, 60)
    mqtt_client.loop_start()

    print(f"Waiting for position fix on me ({MY_ID}) and target ({TARGET_ID})...")
    while MY_ID not in robots_state or TARGET_ID not in robots_state:
        time.sleep(0.1)
    me0, tg0 = robots_state[MY_ID], robots_state[TARGET_ID]
    print(f"Me:     ({me0['x']:.2f}, {me0['y']:.2f}) angle={me0['angle']:.0f}°")
    print(f"Target: ({tg0['x']:.2f}, {tg0['y']:.2f})")

    pipuck = PiPuck(epuck_version=2)
    set_lights(pipuck, False)
    at_target = False  # hysteresis flag for stop/resume

    try:
        while True:
            me  = robots_state.get(MY_ID)
            tgt = robots_state.get(TARGET_ID)

            if not fresh(me) or not fresh(tgt):
                pipuck.epuck.set_motor_speeds(0, 0)
                time.sleep(0.05)
                continue

            mx, my, ma = me["x"], me["y"], me["angle"]
            tx, ty     = tgt["x"], tgt["y"]
            dist = math.hypot(tx - mx, ty - my)

            # Stop + light up when we've caught the target. Use hysteresis so
            # tiny tracking jitter doesn't flicker the lights/motors.
            if at_target:
                if dist < RESUME_DISTANCE:
                    pipuck.epuck.set_motor_speeds(0, 0)
                    time.sleep(0.05)
                    continue
                print(f"  target moved away (dist={dist:.2f}) — resuming chase")
                set_lights(pipuck, False)
                at_target = False
            elif dist <= FOLLOW_DISTANCE:
                print(f"  caught target (dist={dist:.2f}) — lights on, holding")
                set_lights(pipuck, True)
                pipuck.epuck.set_motor_speeds(0, 0)
                at_target = True
                time.sleep(0.05)
                continue

            aim_x, aim_y, blocker = aim_with_avoidance(mx, my, tx, ty)
            hdg = heading_to(aim_x - mx, aim_y - my)
            err = angle_diff(hdg, ma)

            # Single-threshold steering — no dead zone. Either spin in place to
            # align, or drive straight. Motors are commanded every iteration.
            if abs(err) > ANGLE_TURN_THRESHOLD:
                if err > 0:
                    left, right = SPEED_TURN, -SPEED_TURN
                else:
                    left, right = -SPEED_TURN, SPEED_TURN
            else:
                left, right = SPEED_BASE, SPEED_BASE
            pipuck.epuck.set_motor_speeds(left, right)

            tag = f" avoiding {blocker}" if blocker else ""
            print(f"  pos=({mx:.2f},{my:.2f}) tgt=({tx:.2f},{ty:.2f}) "
                  f"dist={dist:.2f} hdg={hdg:.0f}° robot={ma:.0f}° "
                  f"err={err:+.0f}° motors=({left},{right}){tag}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        pipuck.epuck.set_motor_speeds(0, 0)
        set_lights(pipuck, False)
        mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
