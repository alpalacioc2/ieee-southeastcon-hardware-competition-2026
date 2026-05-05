from djitellopy import Tello
import time


def takeoff_and_land(flight_time=2):
    tello = Tello()

    print("Connecting to Tello...")
    tello.connect()

    battery = tello.get_battery()
    print(f"Connected. Battery: {battery}%")

    print("Taking off...")
    tello.takeoff()

    time.sleep(flight_time)

    print("Landing...")
    tello.land()

    print("Done.")
