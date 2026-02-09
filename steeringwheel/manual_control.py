import sys
import termios
import tty
import time
import select

STEER_CENTER = 90
STEER_STEP   = 5
SPEED_STEP   = 10
MAX_SPEED    = 200

steer = STEER_CENTER
speed = 0


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

def clamp(val, min_v, max_v):
    return max(min_v, min(max_v, val))


def start_manual(hub):
    global steer, speed

    print(">>> MANUAL CONTROL READY")
    print("W/S: speed  A/D: steer  SPACE: stop")

    old_term = setup_terminal()

    try:
        while True:
            key = read_key_nonblock()
            if key:
                if key == 'w':
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

            # 🔥 CHỈ GỬI Ý ĐỊNH
            hub.send_manual(steer, speed)

            time.sleep(0.01)

    finally:
        restore_terminal(old_term)
