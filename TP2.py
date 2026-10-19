from pymavlink import mavutil
import csv, math
from datetime import datetime

CONNECTION = 'udpin:0.0.0.0:14550'   # or '/dev/ttyUSB0' for a telemetry radio
BAUD = 57600                         # only used for serial connections
LOG_FILE = 'drone_track.csv'
MAX_RANGE_M = 500                    # alert if drone goes farther than this from home

def haversine(lat1, lon1, lat2, lon2):
    """Distance in meters between two GPS points."""
    R = 6371000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))

master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
print("Waiting for drone heartbeat...")
master.wait_heartbeat()
print("Drone connected (system", master.target_system, ")")

home = None
try:
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'lat', 'lon', 'alt_m', 'dist_from_home_m'])

        while True:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if msg is None:
                print("No telemetry for 5 s. Check the link.")
                continue

            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000   # mm to m

            if lat == 0 and lon == 0:       # no GPS fix yet
                continue
            if home is None:
                home = (lat, lon)           # first valid fix becomes home

            dist = haversine(home[0], home[1], lat, lon)
            writer.writerow([datetime.now().isoformat(timespec='seconds'),
                             round(lat, 7), round(lon, 7), round(alt, 2), round(dist, 1)])
            f.flush()

            print(f"{lat:.6f}, {lon:.6f} | alt {alt:.1f} m | {dist:.0f} m from home")
            if dist > MAX_RANGE_M:
                print("WARNING: drone is outside the allowed range!")

except KeyboardInterrupt:
    print("Tracking stopped. Log saved to", LOG_FILE)