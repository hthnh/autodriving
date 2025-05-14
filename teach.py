# teach.py
import os
import time
import csv
from picamera2 import Picamera2
import smbus2 # For I2C
import socket # For IPC
# import struct # For packing/unpacking control data if needed (not used in current logic)
import select # For non-blocking socket read
import cv2 # Uncomment if you use cv2.flip and have it installed. Currently commented out.
from PIL import Image # Using PIL to save image, as in your original record_frame

# ---------- IPC SETUP (Unix Domain Socket) ----------
SOCKET_PATH = "/tmp/robot_control_socket" # Path for the socket file

# ---------- IMU GY-87 SETUP ----------
bus = smbus2.SMBus(1) # I2C bus 1 on Raspberry Pi

# MPU6050 (Accelerometer and Gyroscope)
MPU6050_ADDR = 0x68
MPU6050_REG_PWR_MGMT_1 = 0x6B
MPU6050_REG_ACCEL_XOUT_H = 0x3B
MPU6050_REG_GYRO_XOUT_H = 0x43
MPU6050_REG_TEMP_OUT_H = 0x41

# HMC5883L (Magnetometer)
HMC5883L_ADDR = 0x1E
HMC5883L_REG_CONFIG_A = 0x00
HMC5883L_REG_CONFIG_B = 0x01
HMC5883L_REG_MODE = 0x02
HMC5883L_REG_XOUT_H = 0x03
HMC5883L_REG_ZOUT_H = 0x05
HMC5883L_REG_YOUT_H = 0x07

# BMP180 (Barometer/Temperature)
BMP180_ADDR = 0x77
BMP180_REG_CONTROL = 0xF4
BMP180_REG_RESULT_MSB = 0xF6
BMP180_CMD_READ_TEMP = 0x2E
BMP180_CMD_READ_PRESSURE_0 = 0x34

# --- BMP180 Calibration Data ---
AC1, AC2, AC3, B1, B2, MB, MC, MD = 0,0,0,0,0,0,0,0
AC4, AC5, AC6 = 0,0,0
BMP180_CALIBRATION_OK = False # Flag to indicate if calibration data was read successfully



def _read_signed_byte(addr, reg):
    """Reads a signed byte from I2C."""
    try:
        val = bus.read_byte_data(addr, reg)
        return val if val < 128 else val - 256
    except OSError as e:
        # print(f"Warning: I2C signed byte read error from addr {hex(addr)}, reg {hex(reg)}: {e}")
        return None

def _read_unsigned_byte(addr, reg):
    """Reads an unsigned byte from I2C."""
    try:
        return bus.read_byte_data(addr, reg)
    except OSError as e:
        # print(f"Warning: I2C unsigned byte read error from addr {hex(addr)}, reg {hex(reg)}: {e}")
        return None

def _read_signed_word_2c(addr, reg):
    """Reads a signed 16-bit word (big-endian) using 2's complement."""
    try:
        high = bus.read_byte_data(addr, reg)
        low = bus.read_byte_data(addr, reg + 1)
        val = (high << 8) + low
        # S?a l?i nh?: M?c d?nh hm ny d?c Big Endian, nhung n?u c?m bi?n tr? v? Little Endian th c?n d?o l?i
        # Tuy nhin, MPU6050, HMC5883L thu?ng  Big Endian. BMP180 cung v?y khi d?c t?ng byte.
        # Gi? nguy logic ny cho cc hm d?c sensor hi?n c.
        return val if val < 32768 else val - 65536
    except OSError as e:
        # print(f"Warning: I2C signed word read error from addr {hex(addr)}, reg {hex(reg)}: {e}")
        return None

def _read_unsigned_word(addr, reg):
    """Reads an unsigned 16-bit word (big-endian)."""
    try:
        high = bus.read_byte_data(addr, reg)
        low = bus.read_byte_data(addr, reg + 1)
        return (high << 8) + low
    except OSError as e:
        # print(f"Warning: I2C unsigned word read error from addr {hex(addr)}, reg {hex(reg)}: {e}")
        return None

def setup_imu():
    global AC1, AC2, AC3, B1, B2, MB, MC, MD, AC4, AC5, AC6, BMP180_CALIBRATION_OK
    
    print("[INFO] Initializing IMU...")
    # Wake MPU6050
    try:
        bus.write_byte_data(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00)
        print("[INFO] MPU6050 woken up.")
    except Exception as e:
        print(f"[ERROR] Failed to wake MPU6050: {e}")

    # Configure HMC5883L
    try:
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70) # 8-average, 15 Hz
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 0xA0) # Gain (might need adjustment)
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00)   # Continuous measurement
        print("[INFO] HMC5883L configured.")
    except Exception as e:
        print(f"[ERROR] Failed to configure HMC5883L: {e}")

    # Read BMP180 calibration data
    print("[INFO] Reading BMP180 calibration data...")
    try:
        AC1 = _read_signed_word_2c(BMP180_ADDR, 0xAA)
        AC2 = _read_signed_word_2c(BMP180_ADDR, 0xAC)
        AC3 = _read_signed_word_2c(BMP180_ADDR, 0xAE)
        AC4 = _read_unsigned_word(BMP180_ADDR, 0xB0)
        AC5 = _read_unsigned_word(BMP180_ADDR, 0xB2)
        AC6 = _read_unsigned_word(BMP180_ADDR, 0xB4)
        B1 = _read_signed_word_2c(BMP180_ADDR, 0xB6)
        B2 = _read_signed_word_2c(BMP180_ADDR, 0xB8)
        MB = _read_signed_word_2c(BMP180_ADDR, 0xBA)
        MC = _read_signed_word_2c(BMP180_ADDR, 0xBC)
        MD = _read_signed_word_2c(BMP180_ADDR, 0xBE)

        # --- crucial check ---
        # A simple check: if all are zero, something is very wrong.
        # More robust checks would ensure values are within expected ranges for BMP180.
        if all(v == 0 for v in [AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD]):
            print("[ERROR] All BMP180 calibration coefficients are zero! Check I2C connection to BMP180 (addr 0x77).")
            BMP180_CALIBRATION_OK = False
        elif AC5 == 0 or AC6 == 0 or MC == 0 or MD == 0 : # Add checks for specific problematic zeros for division
             print(f"[WARNING] Some critical BMP180 calibration coefficients are zero: AC5={AC5}, AC6={AC6}, MC={MC}, MD={MD}")
             print("This might lead to division by zero. Check sensor and I2C connection.")
            
             BMP180_CALIBRATION_OK = False # Treat as failure if critical coeffs are zero
        else:
            BMP180_CALIBRATION_OK = True
            print("[INFO] BMP180 calibration data read successfully.")
            # Optional: Print them for debugging
            print(f"  AC1={AC1}, AC2={AC2}, AC3={AC3}, AC4={AC4}, AC5={AC5}, AC6={AC6}")
            print(f"  B1={B1}, B2={B2}, MB={MB}, MC={MC}, MD={MD}")

    except Exception as e:
        print(f"[ERROR] Failed to read BMP180 calibration data: {e}")
        BMP180_CALIBRATION_OK = False

def get_imu_data():
    global BMP180_CALIBRATION_OK # Ensure we use the global flag
    imu_payload = {
        "accel_x": 0, "accel_y": 0, "accel_z": 0,
        "gyro_x": 0, "gyro_y": 0, "gyro_z": 0,
        "mag_x": 0, "mag_y": 0, "mag_z": 0,
        "temperature_mpu": 0,
        "pressure": -1, "altitude_bmp": -1, "temperature_bmp": -1 # Default to error values
    }
    try: # MPU6050
        # ... (code Ä‘á»c MPU6050 nhÆ° cÅ©) ...
        imu_payload["accel_x"] = _read_signed_word_2c(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H) / 16384.0
        imu_payload["accel_y"] = _read_signed_word_2c(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H + 2) / 16384.0
        imu_payload["accel_z"] = _read_signed_word_2c(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H + 4) / 16384.0
        imu_payload["gyro_x"] = _read_signed_word_2c(MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H) / 131.0
        imu_payload["gyro_y"] = _read_signed_word_2c(MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 2) / 131.0
        imu_payload["gyro_z"] = _read_signed_word_2c(MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 4) / 131.0
        temp_raw_mpu = _read_signed_word_2c(MPU6050_ADDR, MPU6050_REG_TEMP_OUT_H)
        imu_payload["temperature_mpu"] = temp_raw_mpu / 340.0 + 36.53
    except Exception as e:
        print(f"Error reading MPU6050: {e}")

    try: # HMC5883L
        # ... (code Ä‘á»c HMC5883L nhÆ° cÅ©) ...
        mag_x_raw = _read_signed_word_2c(HMC5883L_ADDR, HMC5883L_REG_XOUT_H)
        mag_z_raw = _read_signed_word_2c(HMC5883L_ADDR, HMC5883L_REG_ZOUT_H) # Z-axis
        mag_y_raw = _read_signed_word_2c(HMC5883L_ADDR, HMC5883L_REG_YOUT_H) # Y-axis
        imu_payload["mag_x"] = mag_x_raw * 0.92 
        imu_payload["mag_y"] = mag_y_raw * 0.92 
        imu_payload["mag_z"] = mag_z_raw * 0.92
    except Exception as e:
        print(f"Error reading HMC5883L: {e}")

    if not BMP180_CALIBRATION_OK:
        print("Skipping BMP180 reading due to calibration failure or invalid coefficients.")
        # Values already defaulted to -1 in imu_payload
    else:
        try: # BMP180 calculations
            # Read uncompensated temperature
            bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP)
            time.sleep(0.005) # Wait for measurement (4.5ms for temp)
            UT = _read_unsigned_word(BMP180_ADDR, BMP180_REG_RESULT_MSB)

            # Read uncompensated pressure (OSS=0 for this example)
            OSS = 0 # Oversampling setting (0,1,2,3)
            bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_PRESSURE_0 + (OSS << 6))

            time.sleep(0.005) # Datasheet: 4.5ms for OSS=0
            
            UP_msb = bus.read_byte_data(BMP180_ADDR, 0xF6)
            UP_lsb = bus.read_byte_data(BMP180_ADDR, 0xF7)
            UP_xlsb = bus.read_byte_data(BMP180_ADDR, 0xF8)
            UP = ((UP_msb << 16) + (UP_lsb << 8) + UP_xlsb) >> (8 - OSS)

   

            X1_temp = (UT - AC6) * AC5 / (2**15) # Renamed to avoid conflict with X1 for pressure
            
            if (X1_temp + MD) == 0:
                print(f"[ERROR] BMP180: Division by zero in temperature calculation (X1_temp + MD == 0).")
                print(f"  UT={UT}, AC6={AC6}, AC5={AC5}, MC={MC}, MD={MD}, X1_temp={X1_temp}")
                # imu_payload["temperature_bmp"] already -1
            else:
                X2_temp = MC * (2**11) / (X1_temp + MD)
                B5 = X1_temp + X2_temp
                true_temp_c = (B5 + 8) / (160.0) # Corrected: (2**4 * 10.0) = 160.0 for temp in C
                imu_payload["temperature_bmp"] = true_temp_c

                # Calculate true pressure (only if temperature calculation was successful)
                B6 = B5 - 4000
                # Denominators in X1, X2 for pressure: (2**11), (2**11) - OK
                X1_p = (B2 * (B6 * B6 / (2**12))) / (2**11) # Renamed
                X2_p = AC2 * B6 / (2**11)
                X3_p = X1_p + X2_p
                # Denominator for B3: 4 - OK
                B3 = (((AC1 * 4 + X3_p) << OSS) + 2) / 4
                # Denominators in X1, X2 for B4: (2**13), (2**16) - OK
                X1_b4 = AC3 * B6 / (2**13)
                X2_b4 = (B1 * (B6 * B6 / (2**12))) / (2**16)
                X3_b4 = ((X1_b4 + X2_b4) + 2) / 4 # Denom 4 - OK
                # Denominator for B4: (2**15) - OK
                B4 = AC4 * (X3_b4 + 32768) / (2**15)
                
                if B4 == 0:
                    print(f"[ERROR] BMP180: Division by zero in pressure calculation (B4 == 0).")
                    # imu_payload["pressure"] already -1
                else:
                    B7 = (UP - B3) * (50000 >> OSS)
                    if B7 < 0x80000000:
                        p = (B7 * 2) / B4
                    else:
                        p = (B7 / B4) * 2
                    
                    X1_final = (p / (2**8)) * (p / (2**8))
                    X1_final = (X1_final * 3038) / (2**16) # Denom (2**16) - OK
                    X2_final = (-7357 * p) / (2**16)    # Denom (2**16) - OK
                    pressure_pa = p + (X1_final + X2_final + 3791) / 16.0 # Denom (2**4) - OK
                    imu_payload["pressure"] = pressure_pa / 100.0 # Convert Pa to hPa (mbar)
            
                    # Calculate altitude
                    # Using standard sea level pressure in hPa to match pressure unit
                    sea_level_pressure_hpa = 1013.25 
                  
                    pressure_for_alt_calc = pressure_pa # Use Pa for 101325.0
                    if pressure_for_alt_calc > 0: # Avoid math error if pressure is invalid
                         imu_payload["altitude_bmp"] = 44330 * (1.0 - pow(pressure_for_alt_calc / 101325.0, 1/5.255))
                    else:
                         imu_payload["altitude_bmp"] = -1


        except ZeroDivisionError: # Catch specific error
            print(f"Error reading BMP180: Float division by zero occurred during calculation.")
            # imu_payload values already defaulted to -1 or previous error state
        except Exception as e:
            print(f"Error processing BMP180 data: {e}")
            # imu_payload values already defaulted to -1

    return imu_payload

# ---------- RECORD SETUP ----------
class Record:
    def __init__(self, save_path):
        self.save_path = save_path
        self.frame_dir = os.path.join(save_path, "frames")
        os.makedirs(self.frame_dir, exist_ok=True)

        self.csv_path = os.path.join(save_path, "data.csv")
        with open(self.csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "frame_file",
                "accel_x", "accel_y", "accel_z",
                "gyro_x", "gyro_y", "gyro_z",
                "mag_x", "mag_y", "mag_z",
                "temp_mpu", "temp_bmp", "pressure_hpa", "altitude_bmp", # Changed pressure to pressure_hpa
                "motor_left_dir", "motor_left_speed",
                "motor_right_dir", "motor_right_speed",
                "servo1_angle", "servo2_angle"
            ])

        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration() # Keep original if resolution is important
        self.picam2.configure(config)
        self.picam2.start()
        print("Picamera2 started.")

    def record_frame(self, imu_data, control_commands):
        timestamp = time.time()
        frame_filename = f"{timestamp:.3f}.jpg" # Using 3 decimal places for timestamp
        frame_path = os.path.join(self.frame_dir, frame_filename)
        
        try:
            frame_array = self.picam2.capture_array("main") # Specify "main" stream

            frame_array = cv2.flip(frame_array, -1) 
            
            img = Image.fromarray(frame_array)
            img.save(frame_path)
        except Exception as e:
            print(f"Error capturing/saving frame: {e}")
            frame_filename = "error"

        cmd_motor_left_dir = control_commands.get("motor_left_dir", 0)
        cmd_motor_left_speed = control_commands.get("motor_left_speed", 0)
        cmd_motor_right_dir = control_commands.get("motor_right_dir", 0)
        cmd_motor_right_speed = control_commands.get("motor_right_speed", 0)
        cmd_servo1_angle = control_commands.get("servo1_angle", 90)
        cmd_servo2_angle = control_commands.get("servo2_angle", 90)

        with open(self.csv_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp, frame_filename,
                imu_data.get("accel_x", 0), imu_data.get("accel_y", 0), imu_data.get("accel_z", 0),
                imu_data.get("gyro_x", 0), imu_data.get("gyro_y", 0), imu_data.get("gyro_z", 0),
                imu_data.get("mag_x", 0), imu_data.get("mag_y", 0), imu_data.get("mag_z", 0),
                imu_data.get("temperature_mpu", 0), 
                imu_data.get("temperature_bmp", -1), # Default to -1 if error
                imu_data.get("pressure", -1),       # Already pressure_hpa
                imu_data.get("altitude_bmp", -1),   # Default to -1 if error
                cmd_motor_left_dir, cmd_motor_left_speed,
                cmd_motor_right_dir, cmd_motor_right_speed,
                cmd_servo1_angle, cmd_servo2_angle
            ])

    def close(self):
        if hasattr(self, 'picam2') and self.picam2:
            self.picam2.stop()
            print("Picamera2 stopped.")

# ---------- MAIN ----------
SAVE_DIR = "dataset_teach"
SESSION_NAME = time.strftime("%Y%m%d_%H%M%S")
SAVE_PATH = os.path.join(SAVE_DIR, SESSION_NAME)

def main_teach():
    # It's better to create SAVE_PATH only if IMU and Camera initialize successfully
    # For now, keep as is.
    os.makedirs(SAVE_PATH, exist_ok=True) 
    
    setup_imu() # Initialize IMU sensors
    
    # Check if BMP180 calibration failed critically. You might choose to exit.
    if not BMP180_CALIBRATION_OK:
        print("[CRITICAL] BMP180 calibration failed. Data for BMP180 will be invalid. Consider stopping.")
        # return # Optionally exit if BMP180 is critical for your application

    recorder = None # Initialize to None
    server_socket = None # Initialize to None
    try:
        recorder = Record(save_path=SAVE_PATH)
        print(f"[INFO] Recording session: {SESSION_NAME}")
        print(f"[INFO] Saving data to: {SAVE_PATH}")
        print(f"[INFO] Waiting for control commands on socket: {SOCKET_PATH}")
        print("[INFO] Press Ctrl+C to stop.")

        if os.path.exists(SOCKET_PATH):
            os.remove(SOCKET_PATH)
        
        server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        server_socket.bind(SOCKET_PATH)
        server_socket.setblocking(False)

        latest_control_commands = {
            "motor_left_dir": 0, "motor_left_speed": 0,
            "motor_right_dir": 0, "motor_right_speed": 0,
            "servo1_angle": 90, "servo2_angle": 90
        }

        loop_count = 0
        while True:
            ready_to_read, _, _ = select.select([server_socket], [], [], 0.01)
            if ready_to_read:
                try:
                    data, _ = server_socket.recvfrom(1024)
                    parts = data.decode().strip().split(':')
                    if len(parts) == 6:
                        latest_control_commands["motor_left_dir"] = int(parts[0])
                        latest_control_commands["motor_left_speed"] = int(parts[1])
                        latest_control_commands["motor_right_dir"] = int(parts[2])
                        latest_control_commands["motor_right_speed"] = int(parts[3])
                        latest_control_commands["servo1_angle"] = int(parts[4])
                        latest_control_commands["servo2_angle"] = int(parts[5])
                except BlockingIOError:
                    pass 
                except Exception as e:
                    print(f"Error receiving/parsing control commands: {e}")
            
            imu_data = get_imu_data()
            recorder.record_frame(imu_data, latest_control_commands)
            
            # Print IMU data periodically for debugging, e.g., every 20 loops (2 seconds if 0.1s sleep)
            if loop_count % 20 == 0:
                print(f"IMU: Temp_BMP={imu_data.get('temperature_bmp', -1):.2f}C, Press_hPa={imu_data.get('pressure', -1):.2f}")

            loop_count += 1
            time.sleep(0.1) # Adjust rate as needed

    except KeyboardInterrupt:
        print("\n[INFO] Recording stopped by user.")
    except Exception as e:
        print(f"[ERROR] An unexpected error occurred in main_teach: {e}")
        import traceback
        traceback.print_exc() # Print full traceback for unexpected errors
    finally:
        print("[INFO] Cleaning up...")
        if recorder: # Check if recorder was successfully initialized
            recorder.close()
        if server_socket: # Check if server_socket was successfully initialized
            server_socket.close()
        if os.path.exists(SOCKET_PATH):
            try:
                os.remove(SOCKET_PATH)
            except OSError as e:
                print(f"Error removing socket file: {e}")
        print("[INFO] Teach phase cleanup complete.")

if __name__ == "__main__":
    # Before starting, it's a good idea to run i2cdetect
    print("Please ensure I2C is enabled and sensors are connected.")
    print("Run 'sudo i2cdetect -y 1' (or -y 0 for old Pi models) to check for sensor addresses:")
    print("  MPU6050: 0x68, HMC5883L: 0x1e, BMP180: 0x77")
    # input("Press Enter to continue if sensors are detected...") # Optional pause
    main_teach()
