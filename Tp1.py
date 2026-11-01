import time
from pymavlink import mavutil

CONNECTION = '/dev/ttyUSB0'      # telemetry radio USB adapter (Windows: 'COM3')
BAUD = 57600                     # match the radio's baud rate
STALE_AFTER_S = 5                # warn if no position update for this long

master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
print("Waiting for drone...")
master.wait_heartbeat(timeout=30)
print("Connected to system", master.target_system)

last_fix, last_time = None, 0.0
try:
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg and (msg.lat or msg.lon):
            lat, lon = msg.lat / 1e7, msg.lon / 1e7
            alt = msg.relative_alt / 1000
            last_fix, last_time = (lat, lon, alt), time.time()
            print(f"{lat:.6f}, {lon:.6f} | alt {alt:.1f} m")
            print(f"  map: https://maps.google.com/?q={lat},{lon}")
        age = time.time() - last_time
        if last_fix and age > STALE_AFTER_S:
            print(f"No update for {age:.0f} s. Last known: {last_fix[0]:.6f}, {last_fix[1]:.6f}")
        time.sleep(1)
except KeyboardInterrupt:
    if last_fix:
        print(f"\nLast known position: {last_fix[0]:.6f}, {last_fix[1]:.6f}")
