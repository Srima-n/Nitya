from pymavlink import mavutil
import math, time

CONNECTION = 'udpin:0.0.0.0:14550'  # or '/dev/ttyAMA0' for a serial link
TARGET_ALT_M = 5.0          # hover height
MAX_ALT_M = 10.0            # hard ceiling: land if exceeded
MAX_RADIUS_M = 30.0         # geofence around home
MIN_BATTERY_PCT = 40        # land if battery drops below this
HOVER_TIME_S = 20           # how long to hover before landing
DRY_RUN = True              # True: checks only, no arming. Keep True until tested with props off.

COPTER_MODES = {'GUIDED': 4, 'LAND': 9}


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def set_mode(master, mode_id):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)


def is_armed(master):
    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def current_mode(master):
    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    return hb.custom_mode if hb else None


def prearm_checks(master):
    """Return (ok, reason). Waits briefly for fresh data."""
    gps = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if gps is None or gps.fix_type < 3:
        return False, "no 3D GPS fix"
    sys = master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if sys is None:
        return False, "no SYS_STATUS data"
    if sys.battery_remaining != -1 and sys.battery_remaining < MIN_BATTERY_PCT:
        return False, f"battery at {sys.battery_remaining}%"
    return True, "ok"


def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    return bool(ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED)


def takeoff(master, alt):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, alt)


def land_and_wait(master):
    print("Landing...")
    set_mode(master, COPTER_MODES['LAND'])
    deadline = time.time() + 60
    while time.time() < deadline:
        if not is_armed(master):
            print("Landed and disarmed.")
            return
    print("Warning: still armed after 60 s. Check the aircraft.")


def main():
    master = mavutil.mavlink_connection(CONNECTION)
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Connected to system", master.target_system)

    ok, reason = prearm_checks(master)
    if not ok:
        print("Pre-arm check failed:", reason)
        return
    print("Pre-arm checks passed.")

    if DRY_RUN:
        print("DRY_RUN is True: not arming. Set it to False to fly.")
        return

    # Home = first valid GPS fix
    home = None
    while home is None:
        pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if pos and (pos.lat or pos.lon):
            home = (pos.lat / 1e7, pos.lon / 1e7)

    set_mode(master, COPTER_MODES['GUIDED'])
    time.sleep(1)
    if not arm(master):
        print("Arming rejected. Check the flight controller's pre-arm messages.")
        return

    print(f"Taking off to {TARGET_ALT_M} m")
    takeoff(master, TARGET_ALT_M)

    hover_start = None
    try:
        while True:
            # Operator took control with the RC transmitter: stop issuing commands
            mode = current_mode(master)
            if mode is not None and mode != COPTER_MODES['GUIDED']:
                print("Mode changed by pilot. Script stopping.")
                return

            pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            batt = master.recv_match(type='SYS_STATUS', blocking=False)
            if pos is None:
                continue

            lat, lon = pos.lat / 1e7, pos.lon / 1e7
            alt = pos.relative_alt / 1000
            dist = haversine(home[0], home[1], lat, lon)

            if batt and batt.battery_remaining != -1 and batt.battery_remaining < MIN_BATTERY_PCT:
                print("Low battery. Landing.")
                break
            if dist > MAX_RADIUS_M:
                print(f"Left geofence ({dist:.0f} m). Landing.")
                break
            if alt > MAX_ALT_M:
                print(f"Above {MAX_ALT_M} m ceiling. Landing.")
                break

            if hover_start is None and alt >= TARGET_ALT_M * 0.9:
                hover_start = time.time()
                print("At hover altitude.")
            if hover_start and time.time() - hover_start >= HOVER_TIME_S:
                print("Hover complete. Landing.")
                break

            print(f"alt {alt:.1f} m | {dist:.1f} m from home")
    except KeyboardInterrupt:
        print("Interrupted by user. Landing.")
    finally:
        land_and_wait(master)


if __name__ == '__main__':
    main()
