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
ANGLE_START_DRIVING = 30
ANGLE_STOP_DRIVING  = 50

# Camera angle convention:
#   "math"    → 0° = right (+x), counter-clockwise positive  (atan2(dy, dx))
#   "compass" → 0° = up   (+y), clockwise positive           (atan2(dx, dy))
ANGLE_CONVENTION = "compass"

# Obstacle avoidance
OBSTACLE_RADIUS       = 0.35   # detect another robot within this range ahead
OBSTACLE_LATERAL      = 0.18   # half-width of the corridor we treat as "in path"
OBSTACLE_STALE_S      = 1.5    # ignore positions older than this
EMERGENCY_RADIUS      = 0.18   # if anyone is this close, hard-stop
DETOUR_INWARD         = 0.30   # how far to step away from the border
DETOUR_PASS_AHEAD     = 0.35   # how far past the obstacle before rejoining perimeter
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
    now = time.time()
    out = []
    for rid, st in robots_state.items():
        if rid == ROBOT_ID:
            continue
        if now - st.get("last_seen", 0) > OBSTACLE_STALE_S:
            continue
        out.append((st["x"], st["y"]))
    return out


def target_heading_cardinal(dx, dy):
    if ANGLE_CONVENTION == "compass":
        raw = math.degrees(math.atan2(dx, dy)) % 360
    else:
        raw = math.degrees(math.atan2(dy, dx)) % 360
    return round(raw / 90) * 90 % 360


def target_heading_free(dx, dy):
    if ANGLE_CONVENTION == "compass":
        return math.degrees(math.atan2(dx, dy)) % 360
    return math.degrees(math.atan2(dy, dx)) % 360


def angle_diff(target, current):
    """Signed shortest difference. Positive = turn CW (compass convention)."""
    return (target - current + 180) % 360 - 180


def segment_geometry(target_idx):
    """Geometry of the perimeter segment ending at WAYPOINTS[target_idx]."""
    a = WAYPOINTS[(target_idx - 1) % len(WAYPOINTS)]
    b = WAYPOINTS[target_idx % len(WAYPOINTS)]
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    L = math.hypot(dx, dy)
    sd = (dx / L, dy / L)
    inward = (-sd[1], sd[0])  # CCW-perpendicular = toward arena interior
    return a, b, sd, inward, L


def obstacle_blocking_segment(my_x, my_y, target_idx):
    """Closest other robot ahead of us in the corridor along the current segment."""
    a, _, sd, _, L = segment_geometry(target_idx)
    my_t = (my_x - a[0]) * sd[0] + (my_y - a[1]) * sd[1]

    closest = None
    closest_d = float("inf")
    for ox, oy in other_robots():
        ot  = (ox - a[0]) * sd[0] + (oy - a[1]) * sd[1]
        if ot <= my_t + 0.02:
            continue
        if ot > L:
            continue
        lx = (ox - a[0]) - ot * sd[0]
        ly = (oy - a[1]) - ot * sd[1]
        if math.hypot(lx, ly) > OBSTACLE_LATERAL:
            continue
        d = math.hypot(ox - my_x, oy - my_y)
        if d > OBSTACLE_RADIUS:
            continue
        if d < closest_d:
            closest_d = d
            closest = (ox, oy)
    return closest


def too_close_any():
    st = my_state()
    if not st:
        return False
    for ox, oy in other_robots():
        if math.hypot(ox - st["x"], oy - st["y"]) < EMERGENCY_RADIUS:
            return True
    return False


def build_detour(my_x, my_y, obstacle, target_idx):
    """Three-waypoint detour: step inward, pass the obstacle, step back to perimeter."""
    a, _, sd, inward, L = segment_geometry(target_idx)
    ox, oy = obstacle
    ot = (ox - a[0]) * sd[0] + (oy - a[1]) * sd[1]
    pass_t = min(ot + DETOUR_PASS_AHEAD, L - SEGMENT_END_BUFFER)

    d1 = (my_x + DETOUR_INWARD * inward[0],
          my_y + DETOUR_INWARD * inward[1])
    d2 = (a[0] + pass_t * sd[0] + DETOUR_INWARD * inward[0],
          a[1] + pass_t * sd[1] + DETOUR_INWARD * inward[1])
    d3 = (a[0] + pass_t * sd[0],
          a[1] + pass_t * sd[1])

    # Clamp to safe interior so we never head past the border margin
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
    in_perimeter = False  # True after we first reach a corner — then cardinal motion kicks in
    detour_queue = []

    print(f"Heading to nearest corner #{target_idx}: {WAYPOINTS[target_idx]}")

    try:
        while True:
            st = my_state()
            if st is None:
                time.sleep(0.05)
                continue
            x, y, angle = st["x"], st["y"], st["angle"]

            # Pick the current driving target
            if detour_queue:
                tx, ty, label = detour_queue[0]
                cardinal = False
                on_detour = True
            else:
                tx, ty = WAYPOINTS[target_idx % len(WAYPOINTS)]
                label = f"perim #{target_idx % len(WAYPOINTS)}"
                cardinal = in_perimeter
                on_detour = False

            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)

            # Reached current target?
            if dist < WAYPOINT_THRESHOLD:
                pipuck.epuck.set_motor_speeds(0, 0)
                print(f"  reached {label} ({x:.2f},{y:.2f})")
                if on_detour:
                    detour_queue.pop(0)
                else:
                    in_perimeter = True
                    target_idx += 1
                continue

            # Hard-stop if anyone is critically close
            if too_close_any():
                pipuck.epuck.set_motor_speeds(0, 0)
                print("  ! emergency stop — another robot too close")
                time.sleep(0.2)
                continue

            # While walking the perimeter, look ahead for blockers and plan a detour
            if not on_detour and in_perimeter:
                obs = obstacle_blocking_segment(x, y, target_idx)
                if obs:
                    print(f"  obstacle at ({obs[0]:.2f},{obs[1]:.2f}) — detouring")
                    detour_queue = build_detour(x, y, obs, target_idx)
                    continue

            # Compute desired heading
            hdg_t = target_heading_cardinal(dx, dy) if cardinal else target_heading_free(dx, dy)
            err = angle_diff(hdg_t, angle)

            if abs(err) > ANGLE_STOP_DRIVING:
                if err > 0:
                    pipuck.epuck.set_motor_speeds(SPEED_TURN, -SPEED_TURN)
                else:
                    pipuck.epuck.set_motor_speeds(-SPEED_TURN, SPEED_TURN)
            elif abs(err) < ANGLE_START_DRIVING:
                pipuck.epuck.set_motor_speeds(SPEED_BASE, SPEED_BASE)

            print(f"  {label} pos=({x:.2f},{y:.2f}) dist={dist:.2f} "
                  f"hdg={hdg_t:.0f}° robot={angle:.0f}° err={err:.1f}°")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        pipuck.epuck.set_motor_speeds(0, 0)
        mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
