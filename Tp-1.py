# drone_voice.py
# Lets a Tello drone respond to spoken or typed commands.
# Safety: test indoors, in an open area, with the propeller guards on.

import re
import sys
import time

import speech_recognition as sr
from djitellopy import Tello

# Map each recognized keyword to a drone action.
# Each action is a function that takes the drone object.
COMMANDS = {
    "takeoff":  lambda d: d.takeoff(),
    "land":     lambda d: d.land(),
    "up":       lambda d: d.move_up(30),
    "down":     lambda d: d.move_down(30),
    "left":     lambda d: d.move_left(30),
    "right":    lambda d: d.move_right(30),
    "forward":  lambda d: d.move_forward(30),
    "back":     lambda d: d.move_back(30),
    "rotate left":  lambda d: d.rotate_counter_clockwise(90),
    "rotate right": lambda d: d.rotate_clockwise(90),
    "flip":     lambda d: d.flip_forward(),
    "stop":     lambda d: d.send_rc_control(0, 0, 0, 0),
    "status":   lambda d: print("Battery:", d.get_battery(), "%"),
}


def parse_command(text):
    """Find the first known command phrase inside a sentence."""
    text = text.lower().strip()
    # Check longer phrases first so "rotate left" wins over "left".
    for phrase in sorted(COMMANDS, key=len, reverse=True):
        if re.search(r"\b" + re.escape(phrase) + r"\b", text):
            return phrase
    return None


def listen():
    """Record one phrase from the microphone and return it as text."""
    recognizer = sr.Recognizer()
    with sr.Microphone() as mic:
        print("Listening...")
        recognizer.adjust_for_ambient_noise(mic, duration=0.5)
        audio = recognizer.listen(mic, timeout=5, phrase_time_limit=4)
    try:
        return recognizer.recognize_google(audio)
    except sr.UnknownValueError:
        return ""
    except sr.RequestError:
        print("Speech service unavailable. Check your internet connection.")
        return ""


def run(drone, command):
    if command is None:
        print("Command not recognized.")
        return True  # keep running
    print("Executing:", command)
    if command == "quit":
        return False
    COMMANDS[command](drone)
    return True


def main():
    # Choose input: "voice" uses the microphone, anything else uses the keyboard.
    mode = sys.argv[1] if len(sys.argv) > 1 else "keyboard"

    drone = Tello()
    drone.connect()  # connect your PC to the drone's Wi-Fi first
    print("Connected. Battery:", drone.get_battery(), "%")

    try:
        while True:
            if mode == "voice":
                text = listen()
                if not text:
                    continue
                print("Heard:", text)
            else:
                text = input("Command (or 'quit'): ")
                if text.strip().lower() == "quit":
                    break

            if "quit" in text.lower():
                break
            if not run(drone, parse_command(text)):
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # Always land on exit so the drone doesn't stay airborne.
        try:
            drone.land()
        except Exception:
            pass
        drone.end()


if __name__ == "__main__":
    main()
