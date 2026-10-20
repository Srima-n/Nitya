from pymavlink import mavutil
import csv, json, math, os, time

SERIAL_PORT = '/dev/ttyAMA0'     # Pi UART wired to the flight controller
BAUD = 921600                    # must match the flight controller's serial port setting
LOG_DIR = '/home/pi/flight_logs'
MIN_STEP_M = 5.0                 # record a point only after moving this far (GPS is noisy at short range)


def haversine(lat1, lon1, lat2, lon2):
    """Distance in meters between two GPS points."""
    R = 6371000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def save_geojson(path_points, filename):
    """Save the track as a GeoJSON LineString (opens in geojson.io or QGIS)."""
    if len(path_points) < 2:
        return
    geojson = {
        "type": "Feature",
        "properties": {"points": len(path_points)},
        "geometry": {
            "type": "LineString",
            "coordinates": path_points,   # [lon, lat, alt]
        },
    }
    with open(filename, 'w') as f:
        json.dump(geojson, f, indent=2)


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(LOG_DIR, f'track_{stamp}.csv')
    geo_path = os.path.join(LOG_DIR, f'track_{stamp}.geojson')

    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD)
    print("Waiting for flight controller heartbeat...")
    master.wait_heartbeat()
    print("Connected to system", master.target_system)

    path_points = []        # [lon, lat, alt] for the GeoJSON file
    last = None             # (lat, lon) of the last recorded point
    total_m = 0.0           # distance travelled so far

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['unix_time', 'lat', 'lon', 'alt_m',
                         'ground_speed_mps', 'heading_deg', 'distance_travelled_m'])
        try:
            while True:
                msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
                if msg is None:
                    print("No position data for 5 s. Check the link.")
                    continue

                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                if lat == 0 and lon == 0:      # no GPS fix yet
                    continue

                alt = msg.relative_alt / 1000  # mm to m

                # velocity components are in cm/s (vx = north, vy = east)
                vx, vy = msg.vx / 100, msg.vy / 100
                speed = math.hypot(vx, vy)
                heading = (math.degrees(math.atan2(vy, vx)) + 360) % 360

                d = 0.0 if last is None else haversine(last[0], last[1], lat, lon)

                # only record a point once the drone has moved far enough
                if last is None or d >= MIN_STEP_M:
                    total_m += d
                    last = (lat, lon)
                    path_points.append([round(lon, 7), round(lat, 7), round(alt, 2)])

                    writer.writerow([
                        round(time.time(), 2),
                        round(lat, 7), round(lon, 7), round(alt, 2),
                        round(speed, 2), round(heading, 1), round(total_m, 1),
                    ])
                    f.flush()
                    os.fsync(f.fileno())    # keep data safe if power is cut

                    print(f"{lat:.6f}, {lon:.6f} | alt {alt:.1f} m | "
                          f"{speed:.1f} m/s | travelled {total_m:.0f} m")

        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            save_geojson(path_points, geo_path)
            print(f"Saved {len(path_points)} points to {csv_path}")
            print(f"Track map: {geo_path}")


if __name__ == '__main__':
    main()