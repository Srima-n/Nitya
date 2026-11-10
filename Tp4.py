"""On/off control for a status or navigation light.

This module switches a single output pin on or off. It has no connection
to flight control. The light starts off, and turns off when closed.

Two backends are provided:
  - SimulatedPin: used by the tests and on a laptop, no hardware needed.
  - GpioPin: drives a Raspberry Pi GPIO pin through RPi.GPIO.
    RPi.GPIO is imported only when this backend is used.
"""


class SimulatedPin:
    """Stores the pin state in memory. Used for testing."""

    def __init__(self):
        self.level = False

    def write(self, on: bool):
        self.level = bool(on)

    def close(self):
        self.level = False


class GpioPin:
    """Drives a physical pin on a Raspberry Pi."""

    def __init__(self, pin: int):
        import RPi.GPIO as GPIO  # only available on the Pi
        self._GPIO = GPIO
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

    def write(self, on: bool):
        self._GPIO.output(self.pin, self._GPIO.HIGH if on else self._GPIO.LOW)

    def close(self):
        self._GPIO.output(self.pin, self._GPIO.LOW)
        self._GPIO.cleanup(self.pin)


class Light:
    def __init__(self, name: str, pin):
        self.name = name
        self._pin = pin
        self._on = False
        self._pin.write(False)

    @property
    def is_on(self) -> bool:
        return self._on

    def turn_on(self):
        self._pin.write(True)
        self._on = True

    def turn_off(self):
        self._pin.write(False)
        self._on = False

    def toggle(self):
        if self._on:
            self.turn_off()
        else:
            self.turn_on()

    def close(self):
        """Turn the light off and release the pin."""
        self.turn_off()
        self._pin.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
