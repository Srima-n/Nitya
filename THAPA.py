from djitellopy import Tello
import time

tello = Tello()

try:
    tello.connect()
    print(f"Battery: {tello.get_battery()}%")

    if tello.get_battery() < 30:
        raise RuntimeError("Battery too low to fly safely. Charge it first.")

    tello.takeoff()
    time.sleep(2)

    tello.move_up(50)            # move up 50 cm
    time.sleep(1)
    tello.rotate_clockwise(90)   # turn 90 degrees
    time.sleep(1)
    tello.move_forward(100)      # move forward 100 cm
    time.sleep(1)

    print("Height:", tello.get_height(), "cm")

except Exception as e:
    print("Error:", e)
    tello.emergency()            # cut motors immediately if something goes wrong

finally:
    try:
        tello.land()
    except Exception:
        pass
    tello.end()