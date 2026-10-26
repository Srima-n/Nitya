import time
import requests
import RPi.GPIO as GPIO   # Raspberry Pi GPIO library

# ── Settings ───────────────────────────────────────────────
LATITUDE = 13.08           # set to the drone's base location
LONGITUDE = 80.27
RED_PIN = 17               # GPIO pins wired to the LED driver (through a transistor/MOSFET)
GREEN_PIN = 27
BLUE_PIN = 22
CHECK_EVERY_S = 300        # re-check weather every 5 minutes

# Weather codes (WMO) treated as "harsh": thunderstorm, heavy rain/snow, fog, hail
HARSH_CODES = {45, 48, 65, 67, 75, 77, 82, 86, 95, 96, 99}
HARSH_WIND_KMH = 35        # also treat strong wind as harsh


GPIO.setmode(GPIO.BCM)
for pin in (RED_PIN, GREEN_PIN, BLUE_PIN):
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)


def set_color(r, g, b):
    """Drive the LED. Pins are on/off only, so a full color is 0 or 1 per channel."""
    GPIO.output(RED_PIN, r)
    GPIO.output(GREEN_PIN, g)
    GPIO.output(BLUE_PIN, b)


def is_harsh_weather():
    """Return (harsh, description) from current conditions."""
    url = ('https://api.open-meteo.com/v1/forecast'
           f'?latitude={LATITUDE}&longitude={LONGITUDE}'
           '&current=weather_code,wind_speed_10m')
    try:
        data = requests.get(url, timeout=10).json()['current']
    except (requests.RequestException, KeyError, ValueError):
        return True, "weather unavailable (treating as harsh)"

    code = data['weather_code']
    wind = data['wind_speed_10m']
    harsh = code in HARSH_CODES or wind >= HARSH_WIND_KMH
    return harsh, f"code {code}, wind {wind} km/h"


def main():
    try:
        while True:
            harsh, info = is_harsh_weather()
            if harsh:
                set_color(1, 0, 0)     # red
                state = "HARSH - red"
            else:
                set_color(1, 1, 1)     # white
                state = "GOOD - white"
            print(f"{time.strftime('%H:%M:%S')} {state} ({info})")
            time.sleep(CHECK_EVERY_S)
    except KeyboardInterrupt:
        pass
    finally:
        set_color(0, 0, 0)
        GPIO.cleanup()


if __name__ == '__main__':
    main()
