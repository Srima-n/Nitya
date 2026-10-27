import json, os, time
import requests
from pymavlink import mavutil

# ── Settings ───────────────────────────────────────────────
CONNECTION = '/dev/ttyAMA0'         # or 'udpin:0.0.0.0:14550'
BAUD = 921600
LOW_BATT_PCT = 25                   # first alert: start sending location
CRITICAL_BATT_PCT = 15              # final alert: send again, more urgently
RESEND_EVERY_S = 30                 # repeat alerts while the link is up
SAVE_EVERY_S = 2                    # how often the last fix is saved to disk
LAST_FIX_FILE = 'last_known_position.json'

TG_TOKEN = os.environ.get('TELEGRAM_TOKEN', '')   # from @BotFather
TG_CHAT_ID = os.environ.get('TELEGRAM_CHAT_ID', '')

_last_fix = None           # latest valid fix in memory
_last_saved = 0.0


def save_fix_to_disk(fix):
    """Write the last known fix so it survives a sudden power cut."""
    with open(LAST_FIX_FILE, 'w') as f:
        json.dump(fix, f)
        f.flush()
        os.fsync(f.fileno())


def send_telegram(text, lat=None, lon=None):
    """Send a message, and a map pin if coordinates are known. Returns True on success."""
    if not TG_TOKEN or not TG_CHAT_ID:
        print("Telegram not configured:", text)
        return False
    base = f"https://api.telegram.org/bot{TG_TOKEN}"
    try:
        requests.post(f"{base}/sendMessage",
                      data={'chat_id': TG_CHAT_ID, 'text': text}, timeout=5).raise_for_status()
        if lat is not None and lon is not None:
            requests.post(f"{base}/sendLocation",
                          data={'chat_id': TG_CHAT_ID, 'latitude': lat, 'longitude': lon},
                          timeout=5).raise_for_status()
        return True
    except requests.RequestException as e:
        print("Send failed:", e)
        return False


def alert_text(level, fix, batt):
    lat, lon, alt = fix['lat'], fix['lon'], fix['alt_m']
    return (f"DRONE {level}\n"
            f"Battery: {batt}%\n"
            f"Last position: {lat:.6f}, {lon:.6f}\n"
            f"Altitude: {alt:.1f} m\n"
            f"Map: https://maps.google.com/?q={lat},{lon}\n"
            f"Fix time: {time.strftime('%H:%M:%S', time.localtime(fix['time']))}")


def main():
    global _last_fix, _last_saved

    # Load a previous fix, in case the script restarts after a power loss
    if os.path.exists(LAST_FIX_FILE):
        with open(LAST_FIX_FILE) as f:
            _last_fix = json.load(f)
        print("Loaded saved position:", _last_fix)

    master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
    print("Waiting for flight controller heartbeat...")
    master.wait_heartbeat()
    print("Connected to system", master.target_system)

    batt_pct = None
    alerted_low = False
    alerted_critical = False
    last_send = 0.0

    while True:
        msg = master.recv_match(blocking=True, timeout=2)
        if msg is None:
            # Telemetry gap: keep alerting with the last known fix if the battery is low
            pass
        elif msg.get_type() == 'SYS_STATUS':
            batt_pct = msg.battery_remaining if msg.battery_remaining != -1 else batt_pct
        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            lat, lon = msg.lat / 1e7, msg.lon / 1e7
            if lat != 0 or lon != 0:              # ignore "no GPS fix" zeros
                _last_fix = {'lat': round(lat, 7), 'lon': round(lon, 7),
                             'alt_m': round(msg.relative_alt / 1000, 2),
                             'time': time.time()}
                now = time.time()
                if now - _last_saved >= SAVE_EVERY_S:
                    save_fix_to_disk(_last_fix)
                    _last_saved = now

        # Battery alerts
        if batt_pct is not None and _last_fix is not None:
            now = time.time()
            due = now - last_send >= RESEND_EVERY_S

            if batt_pct <= CRITICAL_BATT_PCT and (not alerted_critical or due):
                ok = send_telegram(alert_text('CRITICAL', _last_fix, batt_pct),
                                   _last_fix['lat'], _last_fix['lon'])
                alerted_critical = True
                last_send = now if ok else last_send

            elif batt_pct <= LOW_BATT_PCT and (not alerted_low or due):
                ok = send_telegram(alert_text('LOW BATTERY', _last_fix, batt_pct),
                                   _last_fix['lat'], _last_fix['lon'])
                alerted_low = True
                last_send = now if ok else last_send

        # Make sure the latest fix is on disk even if no alert has fired
        if _last_fix and time.time() - _last_saved >= SAVE_EVERY_S:
            save_fix_to_disk(_last_fix)
            _last_saved = time.time()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        if _last_fix:
            save_fix_to_disk(_last_fix)
        print("\nStopped.")
      
