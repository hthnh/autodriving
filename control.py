# -*- coding: utf-8 -*-
from RF24 import RF24, RF24_PA_MAX, RF24_250KBPS
import struct
import time
import serial
import sys
import math # Needed for mapping/clamping
import smbus2
import socket
import os
# --- NRF24L01 Configuration ---
radio = RF24(25, 0) # CE, CSN pins (change if needed)
data_format = '<' + ('?' * 11) + ('h' * 4) # 11 bools + 4 int16
data_size = struct.calcsize(data_format)
pipe_address = b'21533' # Pipe address must match sender

# --- Serial to Master Arduino Configuration ---
SERIAL_PORT = '/dev/serial0' # Example: RPi. Change to your port ('COM3', etc.)
BAUD_RATE = 9600
SERIAL_TIMEOUT = 0.1
ser = None

# --- Control Constants ---
# Motor
STOP = 0
FORWARD = 1
BACKWARD = 2
# Speed Levels (Index 0=Stop, 1=Lowest, 5=Highest)
SPEED_LEVELS = [0, 60, 100, 150, 200, 255] # Adjusted speeds, feel free to tune
# Joystick
JOY_MIN = 0
JOY_MAX = 10
JOY_CENTER = 5
# Servo
SERVO_MIN_ANGLE0 = 0
SERVO_MAX_ANGLE0 = 180

SERVO_MIN_ANGLE1 = 35
SERVO_MAX_ANGLE1 = 135
SERVO_CENTER_ANGLE = 90
# Buttons (Indices in the received 'buttons' tuple)
EMERGENCY_STOP_BTN_IDX = 0 # Button 1
RESET_GIMBAL_BTN_IDX = 1 # Button 2

# --- State Variables ---
prev_motor_state = [(STOP, 0), (STOP, 0)] # [(left_dir, left_spd), (right_dir, right_spd)]
prev_servo_state = [SERVO_CENTER_ANGLE, SERVO_CENTER_ANGLE] # [servo1_angle, servo2_angle]


# ---------- IPC CLIENT SETUP (Unix Domain Socket) ----------
TEACH_SOCKET_PATH = "/tmp/robot_control_socket" # Phải giống với teach.py
ipc_client_socket = None



# --- Helper Functions ---
def setup_radio():
    """Initialize and configure NRF24L01 module."""
    if not radio.begin():
        raise RuntimeError("Radio hardware is not responding")
    radio.setChannel(115)
    radio.setPALevel(RF24_PA_MAX)
    radio.setDataRate(RF24_250KBPS)
    radio.openReadingPipe(1, pipe_address)
    radio.startListening()
    print("Radio receiver is ready...")
    radio.printDetails()

def setup_serial():
    """Initialize Serial connection to Master Arduino."""
    global ser
    try:
        print(f"Connecting to Master Arduino on {SERIAL_PORT} at {BAUD_RATE} baud...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(2) # Wait for Arduino reset
        initial_output = ""
        start_time = time.time()
        while time.time() - start_time < 3:
            if ser.in_waiting > 0:
                try: initial_output += ser.read(ser.in_waiting).decode('utf-8', errors='replace')
                except: pass
            time.sleep(0.05)
        print("--- Initial Arduino Output ---")
        print(initial_output.strip())
        print("----------------------------")
        print("Serial connection to Master established.")
        return True
    except serial.SerialException as e:
        print(f"Error connecting to {SERIAL_PORT}: {e}")
        print("Please check port, permissions, and connection.")
        ser = None
        return False
    except Exception as e:
        print(f"An unexpected error occurred during serial setup: {e}")
        ser = None
        return False

def send_serial_command(command):
    """Send command to Master Arduino via Serial."""
    if ser and ser.is_open:
        try:
            # print(f"Sending Serial: {command}") # Uncomment to debug
            ser.write(command.encode('utf-8') + b'\n')
            time.sleep(0.01)
        except serial.SerialException as e: print(f"Serial write error: {e}")
        except Exception as e: print(f"Error sending command '{command}': {e}")
    else: pass # print("Serial port not available.")

def map_range(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another."""
    # Avoid division by zero
    if in_min == in_max:
        return out_min # Or out_max, or average, depending on desired behavior
    # Map value
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# --- Main Processing Logic ---
def process_nrf_data(buttons, joyX1, joyY1, joyX2, joyY2):
    """Process received NRF data and send commands."""
    global prev_motor_state, prev_servo_state

    e_stop_active = buttons[EMERGENCY_STOP_BTN_IDX]
    reset_gimbal_active = buttons[RESET_GIMBAL_BTN_IDX]

    # --- Motor Control ---
    if e_stop_active:
        # Send stop command only if not already stopped
        if prev_motor_state[0] != (STOP, 0):
            send_serial_command("9:MOTOR:0:0:0")
            prev_motor_state[0] = (STOP, 0)
        if prev_motor_state[1] != (STOP, 0):
            send_serial_command("9:MOTOR:1:0:0")
            prev_motor_state[1] = (STOP, 0)
    else:
        # Differential Steering Calculation
        # Normalize joystick values to -1.0 to +1.0 range
        # Forward Y (0) maps to +1.0, Backward Y (10) maps to -1.0
        # Left X (0) maps to -1.0, Right X (10) maps to +1.0
        norm_y = map_range(joyY1, JOY_MIN, JOY_MAX, 1.0, -1.0)
        norm_x = map_range(joyX1, JOY_MIN, JOY_MAX, -1.0, 1.0)

        # Simple Mixing
        raw_left = norm_y - norm_x
        raw_right = norm_y + norm_x

        # Clamp values to ensure they stay within -1.0 to +1.0
        clamped_left = max(-1.0, min(1.0, raw_left))
        clamped_right = max(-1.0, min(1.0, raw_right))

        # Determine direction and speed level
        left_dir = FORWARD if clamped_left >= 0 else BACKWARD
        right_dir = FORWARD if clamped_right >= 0 else BACKWARD

        # Calculate speed level index (0-5) based on absolute value
        # Any movement maps to at least level 1 speed
        left_level_idx = math.ceil(abs(clamped_left) * (len(SPEED_LEVELS) - 1))
        right_level_idx = math.ceil(abs(clamped_right) * (len(SPEED_LEVELS) - 1))

        # Get speed from the levels list
        left_speed = SPEED_LEVELS[left_level_idx]
        right_speed = SPEED_LEVELS[right_level_idx]

        # If level index is 0, ensure direction is STOP
        if left_level_idx == 0: left_dir = STOP
        if right_level_idx == 0: right_dir = STOP

        # Send commands if motor state changed
        current_left_state = (left_dir, left_speed)
        current_right_state = (right_dir, right_speed)

        if current_left_state != prev_motor_state[0]:
            cmd = f"9:MOTOR:0:{left_dir}:{left_speed}"
            send_serial_command(cmd)
            prev_motor_state[0] = current_left_state
            # print(f"Left Motor State Changed: Dir={left_dir}, Spd={left_speed}") # Debug

        if current_right_state != prev_motor_state[1]:
            cmd = f"9:MOTOR:1:{right_dir}:{right_speed}"
            send_serial_command(cmd)
            prev_motor_state[1] = current_right_state
            # print(f"Right Motor State Changed: Dir={right_dir}, Spd={right_speed}") # Debug

    # --- Servo Control ---
    if reset_gimbal_active:
        target_servo1_angle = SERVO_CENTER_ANGLE
        target_servo2_angle = SERVO_CENTER_ANGLE
    else:
        # Map Joystick 2 to servo angles
        target_servo1_angle = int(map_range(joyX2, JOY_MIN, JOY_MAX, SERVO_MIN_ANGLE1, SERVO_MAX_ANGLE1))
        target_servo2_angle = int(map_range(joyY2, JOY_MAX, JOY_MIN, SERVO_MIN_ANGLE0, SERVO_MAX_ANGLE0)) # Assuming Y controls servo 2

        # Clamp angles to ensure they are within valid servo range
        target_servo1_angle = max(SERVO_MIN_ANGLE1, min(SERVO_MAX_ANGLE1, target_servo1_angle))
        target_servo2_angle = max(SERVO_MIN_ANGLE0, min(SERVO_MAX_ANGLE0, target_servo2_angle))

    # Send commands if servo state changed
    if target_servo1_angle != prev_servo_state[0]:
        cmd = f"9:SERVO:1:{target_servo1_angle}"
        send_serial_command(cmd)
        prev_servo_state[0] = target_servo1_angle
        # print(f"Servo 1 State Changed: Angle={target_servo1_angle}") # Debug

    if target_servo2_angle != prev_servo_state[1]:
        cmd = f"9:SERVO:0:{target_servo2_angle}"
        send_serial_command(cmd)
        prev_servo_state[1] = target_servo2_angle
        # print(f"Servo 2 State Changed: Angle={target_servo2_angle}") # Debug
    send_control_data_ipc(left_dir, left_speed, right_dir, right_speed, target_servo1_angle, target_servo2_angle)


def setup_ipc_client():
    global ipc_client_socket
    try:
        ipc_client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        print(f"IPC Client ready to send to {TEACH_SOCKET_PATH}")
        return True
    except Exception as e:
        print(f"Error setting up IPC client: {e}")
        ipc_client_socket = None
        return False

def send_control_data_ipc(left_dir, left_speed, right_dir, right_speed, servo1_angle, servo2_angle):
    if ipc_client_socket:
        try:
            message = f"{left_dir}:{left_speed}:{right_dir}:{right_speed}:{servo1_angle}:{servo2_angle}"
            print(f"IPC Sending: {message}") # Debug
            ipc_client_socket.sendto(message.encode(), TEACH_SOCKET_PATH)
        except Exception as e:
            # Có thể server chưa sẵn sàng, với DGRAM thì sendto không báo lỗi ngay nếu server chưa bind
            print(f"Error sending control data via IPC: {e}")
            pass # Hoặc thử kết nối lại nếu dùng SOCK_STREAM

# ... (setup_radio, setup_serial của bạn cho Arduino Master) ...

# Trong hàm determine_motor_state hoặc process_nrf_data của bạn:
# Sau khi bạn đã tính toán xong target_left_dir, target_left_speed, v.v.
# và *trước hoặc sau khi* gửi lệnh Serial xuống Arduino Master,
# bạn sẽ gọi hàm gửi IPC:

# Ví dụ trong hàm process_nrf_data (hoặc tương đương) của manual_control.py
# ... tính toán target_left_dir, target_left_speed, target_right_dir, target_right_speed ...
# ... tính toán target_servo1_angle, target_servo2_angle ...


def loop():
    """Main loop reading NRF24 and sending Serial commands."""
    while True:
        if radio.available():
            buffer = radio.read(data_size)
            if len(buffer) == data_size:
                try:
                    data = struct.unpack(data_format, buffer)
                    buttons = data[:11]
                    joyX1, joyY1, joyX2, joyY2 = data[11:]
                    print(data)

                    # Process the received data
                    process_nrf_data(buttons, joyX1, joyY1, joyX2, joyY2)

                except struct.error as e:
                    print(f"Error unpacking data: {e}. Buffer length: {len(buffer)}")
                except Exception as e:
                    print(f"Error processing received data: {e}")
            else:
                # radio.flush_rx() # Consider uncommenting if using older library and experiencing issues
                print(f"Warning: Received buffer size mismatch. Expected {data_size}, got {len(buffer)}")

        # Loop delay
        time.sleep(0.02) # ~50Hz


if __name__ == "__main__":
    radio_obj = None # Define radio_obj outside try block for finally clause
    try:
        radio_obj = radio # Assign global radio object
        setup_radio()
        if setup_serial():
            if setup_ipc_client():
                loop()
            else:
                print("Exiting due to IPC client setup failure.")
        else:
            print("Exiting due to serial connection failure.")

    except RuntimeError as e: print(f"Runtime Error: {e}")
    except KeyboardInterrupt: print("\nStopped by user")
    except Exception as e: print(f"An unexpected error occurred in main: {e}")
    finally:
        print("Initiating shutdown sequence...")
        # Safety: Stop motors before exiting
        print("Stopping motors...")
        send_serial_command("9:MOTOR:0:0:0")
        time.sleep(0.05)
        send_serial_command("9:MOTOR:1:0:0")
        time.sleep(0.05)


        if radio_obj is not None : # Check if radio object was initialized
             try:
                  radio_obj.stopListening()
                  print("Radio stopped.")
             except Exception as e:
                  print(f"Error stopping radio: {e}")

        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")
        print("Exiting program.")
        sys.exit(0)
