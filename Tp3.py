from nitya.health import TelemetrySnapshot, check_telemetry

snap = TelemetrySnapshot(timestamp=100.0, now=101.0, gps_fix_type=3,
                         satellites=8, battery_pct=18.0, alt_agl_m=40.0)
for warning in check_telemetry(snap):
    print("WARNING:", warning)
