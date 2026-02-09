# manual_control.py

import sys
import termios
import tty
import time
import select


from protocol import build_packet, CMD_DRIVE, CMD_PING
from uart_driver import UARTDriver


STEER_CENTER = 90
STEER_STEP   = 5
SPEED_STEP   = 10
MAX_SPEED    = 200

steer = STEER_CENTER
speed = 0
SEND_PERIOD = 0.05  # 50 ms = 20 Hz


def setup_terminal():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)
    return old

def restore_terminal(old):
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)

def read_key_nonblock():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None



def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def clamp(val, min_v, max_v):
    return max(min_v, min(max_v, val))


def main():
    global steer, speed

    uart = UARTDriver()
    last_send = 0.0

    print(">>> MANUAL CONTROL STARTED")
    print("W/S: speed  A/D: steer  SPACE: stop  Q: quit")

    old_term = setup_terminal()   # ⭐ SET RAW 1 LẦN

    try:
        # ---- FAST LINK CHECK ----
        uart.send(build_packet(CMD_PING))
        t0 = time.time()
        while time.time() - t0 < 1.0:
            if 0xAC in uart.read(32):
                print("✅ Link OK")
                break

        while True:
            now = time.time()

            key = read_key_nonblock()
            if key:
                if key == 'q':
                    break
                elif key == 'w':
                    speed += SPEED_STEP
                elif key == 's':
                    speed -= SPEED_STEP
                elif key == 'a':
                    steer -= STEER_STEP
                elif key == 'd':
                    steer += STEER_STEP
                elif key == ' ':
                    speed = 0

            speed = clamp(speed, -MAX_SPEED, MAX_SPEED)
            steer = clamp(steer, 45, 135)

            if now - last_send >= SEND_PERIOD:
                last_send = now

                if speed >= 0:
                    dir_l = dir_r = 1
                    spd_l = spd_r = speed
                else:
                    dir_l = dir_r = 2
                    spd_l = spd_r = abs(speed)

                payload = bytes([
                    steer,
                    dir_l, spd_l,
                    dir_r, spd_r
                ])

                uart.send(build_packet(CMD_DRIVE, payload))
                print(f"\rSteer={steer} Speed={speed}   ", end="")

            time.sleep(0.005)

    except KeyboardInterrupt:
        pass

    finally:
        restore_terminal(old_term)   # ⭐ RESTORE CUỐI CHƯƠNG TRÌNH
        print("\nStopping...")
        uart.send(build_packet(
            CMD_DRIVE,
            bytes([STEER_CENTER, 0, 0, 0, 0])
        ))
        uart.close()

if __name__ == "__main__":
    main()