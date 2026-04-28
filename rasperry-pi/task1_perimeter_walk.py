import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

ROBOT_ID = "26"

# Arena: 2.0 m × 1.0 m
ARENA_X, ARENA_Y = 2.0, 1.0
MARGIN = 0.10  # 10 cm safe distance from border

# Perimeter waypoints, traversed counter-clockwise (BL → BR → TR → TL → ...)
WAYPOINTS = [
    (MARGIN,            MARGIN),
    (ARENA_X - MARGIN,  MARGIN),
    (ARENA_X - MARGIN,  ARENA_Y - MARGIN),
    (MARGIN,            ARENA_Y - MARGIN),
]

WAYPOINT_THRESHOLD = 0.08
SPEED_BASE = 500
SPEED_TURN = 400
ANGLE_TURN_THRESHOLD = 30  # if heading error exceeds this, spin in place; else drive straight

# Camera angle convention:
#   "math"    → 0° = right (+x), counter-clockwise positive  (atan2(dy, dx))
#   "compass" → 0° = up   (+y), clockwise positive           (atan2(dx, dy))
ANGLE_CONVENTION = "compass"

# Obstacle avoidance — only react if another robot is genuinely in our way.
OBSTACLE_RADIUS       = 0.22   # only consider robots ahead within this range
OBSTACLE_LATERAL      = 0.10   # robot is "in the path" only if within 10 cm of our line
OBSTACLE_STALE_S      = 1.5    # ignore positions older than this
DETOUR_INWARD         = 0.25   # step this far away from the border to bypass
DETOUR_PASS_AHEAD     = 0.30   # how far past the blocker before returning to perimeter
SEGMENT_END_BUFFER    = 0.05   # don't aim past this close to the next corner

robots_state = {}  # robot_id -> {"x", "y", "angle", "last_seen"}


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


def my_state():
    return robots_state.get(ROBOT_ID)


def other_robots():
    """List of (id, x, y) for every other robot we've seen recently."""
    now = time.time()
    out = []
    for rid, st in robots_state.items():
        if rid == ROBOT_ID:
            continue
        if now - st.get("last_seen", 0) > OBSTACLE_STALE_S:
            continue
        out.append((rid, st["x"], st["y"]))
    return out


def target_heading_cardinal(dx, dy):
    if ANGLE_CONVENTION == "compass":
        raw = math.degrees(math.atan2(dx, dy)) % 360
    else:
        raw = math.degrees(math.atan2(dy, dx)) % 360
    return round(raw / 90) * 90 % 360


def angle_diff(target, current):
    """Signed shortest difference. Positive = turn CW (compass convention)."""
    return (target - current + 180) % 360 - 180


def obstacle_in_path(my_x, my_y, tx, ty):
    """Closest other robot inside the forward corridor between us and (tx, ty).

    Returns (id, x, y) of the blocker or None.
    """
    fdx, fdy = tx - my_x, ty - my_y
    L = math.hypot(fdx, fdy)
    if L < 1e-6:
        return None
    fx, fy = fdx / L, fdy / L

    closest = None
    closest_d = float("inf")
    for rid, ox, oy in other_robots():
        rx, ry = ox - my_x, oy - my_y
        f = rx * fx + ry * fy
        if f <= 0.02 or f > OBSTACLE_RADIUS:
            continue
        if f > L - 0.02:  # past or at the target
            continue
        lx = rx - f * fx
        ly = ry - f * fy
        if math.hypot(lx, ly) > OBSTACLE_LATERAL:
            continue
        d = math.hypot(rx, ry)
        if d < closest_d:
            closest_d = d
            closest = (rid, ox, oy)
    return closest


def inward_perp(my_x, my_y, fx, fy):
    """Pick the perpendicular of (fx, fy) that points toward the arena center."""
    cw  = ( fy, -fx)
    ccw = (-fy,  fx)
    tcx, tcy = (ARENA_X / 2 - my_x), (ARENA_Y / 2 - my_y)
    return ccw if (ccw[0] * tcx + ccw[1] * tcy) >= (cw[0] * tcx + cw[1] * tcy) else cw


def build_detour(my_x, my_y, tx, ty, obstacle):
    """Three-waypoint detour around `obstacle` while heading from (my) toward (tx, ty)."""
    fdx, fdy = tx - my_x, ty - my_y
    L = math.hypot(fdx, fdy)
    if L < 1e-6:
        return []
    fx, fy = fdx / L, fdy / L
    ix, iy = inward_perp(my_x, my_y, fx, fy)

    ox, oy = obstacle
    ot = (ox - my_x) * fx + (oy - my_y) * fy
    pass_t = min(ot + DETOUR_PASS_AHEAD, L - SEGMENT_END_BUFFER)
    pass_t = max(pass_t, ot + 0.05)  # at least nudge past the obstacle

    d1 = (my_x + DETOUR_INWARD * ix,
          my_y + DETOUR_INWARD * iy)
    d2 = (my_x + pass_t * fx + DETOUR_INWARD * ix,
          my_y + pass_t * fy + DETOUR_INWARD * iy)
    d3 = (my_x + pass_t * fx,
          my_y + pass_t * fy)

    def clamp(p):
        return (
            min(max(p[0], MARGIN), ARENA_X - MARGIN),
            min(max(p[1], MARGIN), ARENA_Y - MARGIN),
        )
    return [
        (*clamp(d1), "detour-in"),
        (*clamp(d2), "detour-pass"),
        (*clamp(d3), "detour-out"),
    ]


def nearest_waypoint_index():
    st = my_state()
    return min(
        range(len(WAYPOINTS)),
        key=lambda i: math.hypot(WAYPOINTS[i][0] - st["x"], WAYPOINTS[i][1] - st["y"]),
    )


def main():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(Broker, Port, 60)
    mqtt_client.loop_start()

    print("Waiting for position fix...")
    while my_state() is None:
        time.sleep(0.1)
    st = my_state()
    print(f"Position: ({st['x']:.2f}, {st['y']:.2f})")

    pipuck = PiPuck(epuck_version=2)

    target_idx = nearest_waypoint_index()
    detour_queue = []

    print(f"Heading to nearest corner #{target_idx}: {WAYPOINTS[target_idx]}")

    try:
        while True:
            st = my_state()
            if st is None:
                time.sleep(0.05)
                continue
            x, y, angle = st["x"], st["y"], st["angle"]

            # Pick the current driving target. Heading is always snapped to the nearest
            # cardinal direction so motion stays parallel to the borders, both on the
            # perimeter and during detours (which are always axis-aligned).
            if detour_queue:
                tx, ty, label = detour_queue[0]
                on_detour = True
            else:
                tx, ty = WAYPOINTS[target_idx % len(WAYPOINTS)]
                label = f"perim #{target_idx % len(WAYPOINTS)}"
                on_detour = False

            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)

            # Reached current target? Advance without halting — next iteration will
            # immediately recompute motor commands toward the new target so the robot
            # never sits at zero velocity in a steering dead zone.
            if dist < WAYPOINT_THRESHOLD:
                print(f"  reached {label} ({x:.2f},{y:.2f})")
                if on_detour:
                    detour_queue.pop(0)
                else:
                    target_idx += 1
                continue

            # Look ahead for any other robot in our path and plan a detour around it.
            # Works in any state (perimeter or already-detouring) — if a new blocker
            # appears we just re-plan from current position toward the next perimeter
            # corner. Only triggers when a robot is genuinely in our path: within
            # OBSTACLE_RADIUS ahead and OBSTACLE_LATERAL of our line of motion.
            obs = obstacle_in_path(x, y, tx, ty)
            if obs:
                rid, ox, oy = obs
                final_target = WAYPOINTS[target_idx % len(WAYPOINTS)]
                new_detour = build_detour(x, y, final_target[0], final_target[1], (ox, oy))
                if new_detour:
                    print(f"  robot {rid} at ({ox:.2f},{oy:.2f}) blocking — detouring")
                    detour_queue = new_detour
                    continue

            # Always snap to nearest cardinal so motion stays parallel to the border.
            hdg_t = target_heading_cardinal(dx, dy)
            err = angle_diff(hdg_t, angle)

            # Single-threshold controller: spin in place to align, else drive straight.
            # Always commands motors so we never sit in a dead zone.
            if abs(err) > ANGLE_TURN_THRESHOLD:
                if err > 0:
                    left, right = SPEED_TURN, -SPEED_TURN
                else:
                    left, right = -SPEED_TURN, SPEED_TURN
            else:
                left, right = SPEED_BASE, SPEED_BASE
            pipuck.epuck.set_motor_speeds(left, right)

            print(f"  {label} pos=({x:.2f},{y:.2f}) dist={dist:.2f} "
                  f"hdg={hdg_t:.0f}° robot={angle:.0f}° err={err:.1f}° "
                  f"motors=({left},{right})")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        pipuck.epuck.set_motor_speeds(0, 0)
        mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
