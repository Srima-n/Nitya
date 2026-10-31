# Nitya: Sentinel 550 Drone Perception and Telemetry Platform

A research prototype for a 550 mm quadcopter running ArduCopter on a Pixhawk-class flight controller, with a Raspberry Pi 5 as the companion computer. The project covers telemetry logging, perception on recorded footage, search planning, and a set of safety-gated input and output modules.

**Status: not flight-tested.** The code has been written and checked in parts, including geometry self-checks and offline examples. It has not been run on a real drone, and nothing here should be flown without a qualified pilot, the required permits, and the staged test plan in the build guide.

## Design principle

No module sends flight commands directly. Detection, gesture, and voice modules produce reports or intents. Those must pass the autonomous controller's checks (GPS fix, battery, geofence, flight mode) before anything happens. A misdetection or a misheard word should never be able to start or change a flight.

## What is included

| Area | Description | Status |
|---|---|---|
| Build guide | Wiring, flight controller parameters, perception config, staged test plan | Documented (PDF in repo) |
| Telemetry logging | Records GPS track from the flight controller to CSV and GeoJSON | Written, not flight-tested |
| Autonomous takeoff and landing | Pre-arm checks, geofence, battery and mode monitoring, with a dry-run default | Written, SITL and dry-run only |
| Low-battery location alert | Sends last known position to a Telegram bot, with a local backup file | Written, not tested with a live drone |
| Object detection and tracking | YOLO detector with ByteTrack, label voting, and short-term persistence | Written, needs testing on recorded footage |
| Sighting engine | Projects detections to ground positions, confirms tracks over several frames, de-duplicates | Geometry self-check only |
| Coverage planner | Footprint-based coverage grid, nearest-unseen search, lawnmower waypoints, weighted fusion | Offline example runs |
| Cloud sync | Background uploader with offline outbox and retries | Written, needs a test endpoint |
| Rescue coordinator reporting | Sends confirmed sightings over HTTPS with idempotency keys and dead-letter handling | Written, needs a test server |
| Gesture control | Hand-sign recognition with debouncing and a command gate, simulation mode only | Written, simulation only |
| Voice translation | Speech recognition, language detection, and translation from a microphone | Written, not tested on drone hardware |
| Weather-status LED | Reads public weather data and sets a red or white LED | Written, needs GPIO hardware |
| Fan control | Duty-cycle policy from temperature, battery, and task priority | Written, dry-run only |

## Known limits

- **Position estimates are unmeasured.** Ground projection assumes flat terrain and a calibrated camera. Accuracy has not been measured against surveyed targets.
- **The thermal detector is a baseline.** It flags any warm object. A trained thermal model is needed for reliable results.
- **The fusion error model is a guess.** It should be calibrated against field trial data.
- **Sightings are unverified leads.** Every report is marked as needing human verification. This project is meant to assist trained rescuers, not replace them.
- **Gesture and voice are not safety-validated.** They are prototypes for research only.

## Requirements

- Python 3.10 or newer
- Packages are listed in `requirements.txt`
- A Pixhawk-class flight controller running ArduCopter, for any MAVLink code
- Raspberry Pi 5 or equivalent for the companion computer
- Optional: a thermal camera, GPIO-connected LED and fan hardware

## Setup

```bash
pip install -r requirements.txt
```

Secrets are read from environment variables, never from the code:

```bash
export FIREBASE_SECRET=...
export RESCUE_COORD_TOKEN=...
export TELEGRAM_TOKEN=...
export TELEGRAM_CHAT_ID=...
```

Run the geometry self-check before anything else:

```bash
python3 sighting.py --selftest
```

## Testing order

Follow the staged plan in the build guide. Keep all `DRY_RUN` flags enabled until each stage passes.

1. Simulation (ArduCopter SITL on a laptop)
2. Bench test with propellers removed
3. Sensors and logging while stationary
4. Perception on recorded footage
5. Tethered hover, with the RC transmitter in hand
6. Free flight, only in a permitted area with a qualified pilot

## Safety

- Keep the RC transmitter in hand during any flight test.
- Verify all failsafes in the ground station before every flight day.
- Check local drone regulations for autonomous flight, flight over people, and data collection.
- Have a qualified drone pilot review all flight-related code before flight.

## Licence and use

This is an educational and research prototype. Do not use the location-tracking or person-detection features to monitor people without their knowledge or consent.
