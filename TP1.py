from pymavlink import mavutil

# Connect over UDP (SITL or telemetry bridge)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected to system", master.target_system)

# Read telemetry
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
print("Lat:", msg.lat / 1e7, "Lon:", msg.lon / 1e7, "Alt:", msg.relative_alt / 1000, "m")

# Arm the motors (test on the bench with props removed first)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    1, 0, 0, 0, 0, 0, 0)