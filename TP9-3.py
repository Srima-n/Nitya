import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

# ── Settings ───────────────────────────────────────────────
FAN_PIN = 18                  # GPIO pin driving the fan's MOSFET/transistor (PWM)
PWM_HZ = 25
POLL_S = 2.0
MIN_CHANGE_PCT = 5            # ignore duty changes smaller than this (avoids fan chatter)
MAX_STEP_PCT = 10             # limit how fast duty can change per poll
DRY_RUN = True                # True = print only, no GPIO


class Priority(IntEnum):
    LOW = 0        # idle, logging only
    NORMAL = 1     # routine telemetry
    HIGH = 2       # active detection or tracking
    CRITICAL = 3   # active search, or a mission-critical link


PRIORITY_DEMAND = {Priority.LOW: 20, Priority.NORMAL: 50,
                   Priority.HIGH: 70, Priority.CRITICAL: 100}


@dataclass
class PowerState:
    battery_pct: Optional[float]   # None if unknown; treated as normal
    cpu_temp_c: Optional[float]    # None if unreadable; thermal floor is skipped
    priority: Priority


# ── Sensors ────────────────────────────────────────────────
def read_cpu_temp() -> Optional[float]:
    """Raspberry Pi / most Linux boards expose temperature here (milli-degrees C)."""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return int(f.read().strip()) / 1000.0
    except OSError:
        return None


def read_battery_pct(master) -> Optional[float]:
    """Battery from MAVLink SYS_STATUS. Pass the pymavlink connection, or None to skip."""
    if master is None:
        return None
    msg = master.recv_match(type='SYS_STATUS', blocking=False)
    if msg and msg.battery_remaining != -1:
        return float(msg.battery_remaining)
    return None


# ── Policy ─────────────────────────────────────────────────
def thermal_floor(temp_c: Optional[float]) -> float:
    """Minimum duty the temperature requires. Battery savings never go below this."""
    if temp_c is None:
        return 0.0
    if temp_c >= 75:
        return 100.0
    if temp_c >= 65:
        return 70.0
    if temp_c >= 55:
        return 40.0
    return 0.0


def battery_cap(batt_pct: Optional[float]) -> float:
    """Highest duty
