import time
import os
import sys
import keyboard

sys.path.append("..")

from low_level.client import LowLevelClient


PORT = "/dev/serial0"
# Test trên Windows thì đổi thành:
# PORT = "COM3"


speed = 0.0
steer = 0.0

SPEED_STEP = 0.02
STEER_STEP = 0.05


def clear():
    os.system("clear" if os.name != "nt" else "cls")


def main():
    global speed, steer

    car = LowLevelClient(
        port=PORT,
        baud=115200,
        tx_hz=20,
    )

    car.start()

    try:
        while True:
            if keyboard.is_pressed("q"):
                break

            if keyboard.is_pressed("space"):
                speed = 0.0

            if keyboard.is_pressed("w"):
                speed += SPEED_STEP
            elif keyboard.is_pressed("s"):
                speed -= SPEED_STEP

            if keyboard.is_pressed("a"):
                steer -= STEER_STEP
            elif keyboard.is_pressed("d"):
                steer += STEER_STEP
            else:
                if steer > 0:
                    steer -= 0.02
                elif steer < 0:
                    steer += 0.02

            speed = max(-1.0, min(1.0, speed))
            steer = max(-1.0, min(1.0, steer))

            if abs(steer) < 0.03:
                steer = 0.0

            car.set_intent(
                speed=speed,
                steer=steer,
                brake=False,
            )

            tel = car.get_telemetry()

            clear()
            print("AUTODRIVE V2 MANUAL TEST")
            print("=" * 40)
            print("W/S speed | A/D steer | SPACE stop | Q quit")
            print()
            print(f"CMD speed={speed:+.2f} steer={steer:+.2f}")
            print("-" * 40)
            print(f"STATE : {tel['state']}")
            print(f"RPM   : {tel['rpm']}")
            print(f"TARGET: {tel['target']}")
            print(f"RAMP  : {tel['ramp']}")
            print(f"PWM   : {tel['pwm']}")
            print(f"DIST  : {tel['dist']}")
            print(f"HB    : {tel['hb']}")
            print("-" * 40)
            print(tel["raw"])

            time.sleep(0.05)

    finally:
        car.stop()


if __name__ == "__main__":
    main()