# repeat.py
# Reads a taught session, applies IMU-based correction using MadgwickAHRS,
# and publishes final commands to Redis for uart_sender.py

import os
import time
import csv
import smbus # For I2C with IMU
import redis # For publishing commands
import math
import numpy as np
import sys   # For command line arguments and exit
from ahrs.filters import Madgwick # Assuming you installed this library

# --- Configuration ---
SERVO_CENTER_ANGLE = 90 # Default servo position if not specified in log

# Redis Configuration (for publishing processed commands)
REDIS_HOST = 'localhost'
REDIS_PORT = 6379
redis_client_publisher = None
PROCESSED_COMMAND_STREAM_NAME = 'vehicle:commands_processed'

# IMU GY-87 Global Variables & Constants
bus = smbus.SMBus(1) # I2C bus 1 on Raspberry Pi
MPU6050_ADDR = 0x68
MPU6050_REG_PWR_MGMT_1 = 0x6B
MPU6050_REG_ACCEL_XOUT_H = 0x3B
MPU6050_REG_GYRO_XOUT_H = 0x43
MPU6050_REG_TEMP_OUT_H = 0x41

HMC5883L_ADDR = 0x1E
HMC5883L_REG_CONFIG_A = 0x00
HMC5883L_REG_CONFIG_B = 0x01
HMC5883L_REG_MODE = 0x02
HMC5883L_REG_DATA_X_MSB = 0x03

BMP180_ADDR = 0x77
BMP180_REG_CONTROL = 0xF4
BMP180_REG_RESULT_MSB = 0xF6
BMP180_CMD_READ_TEMP = 0x2E
BMP180_CMD_READ_PRESSURE_0 = 0x34

# BMP180 Calibration data
AC1, AC2, AC3, B1, B2, MB, MC, MD = 0,0,0,0,0,0,0,0
AC4, AC5, AC6 = 0,0,0

# --- Madgwick Filter Setup ---
# You might need to tune the beta gain.
# If your MadgwickAHRS library expects sample_period, calculate it from your loop rate.
SAMPLE_RATE = 50.0  # Hz (Tần suất bạn lấy mẫu IMU và gọi update)
SAMPLE_PERIOD = 1.0 / SAMPLE_RATE
try:
    # Thử khởi tạo với sample_period nếu thư viện hỗ trợ
    ahrs_filter = Madgwick(sample_period=SAMPLE_PERIOD, beta=0.1)
except TypeError:
    # Nếu không, khởi tạo không có sample_period và sẽ truyền dt vào update
    ahrs_filter = Madgwick(beta=0.1) # Hoặc Madgwick(gain=0.1) tùy thư viện

last_ahrs_update_time = 0.0
current_quaternion_ahrs = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z - DẠNG NUMPY ARRAY


# --- IMU Helper Functions (Your complete functions go here) ---
def read_signed_word(bus_obj, addr, reg):
    high = bus_obj.read_byte_data(addr, reg)
    low = bus_obj.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    return value - 65536 if value >= 0x8000 else value

def read_unsigned_word(bus_obj, addr, reg):
    high = bus_obj.read_byte_data(addr, reg)
    low = bus_obj.read_byte_data(addr, reg + 1)
    return (high << 8) + low

def read_signed_word_big_endian(bus_obj, addr, reg_msb):
    msb = bus_obj.read_byte_data(addr, reg_msb)
    lsb = bus_obj.read_byte_data(addr, reg_msb + 1)
    val = (msb << 8) + lsb
    return val - 65536 if val >= 0x8000 else val

def read_unsigned_word_big_endian(bus_obj, addr, reg_msb):
    msb = bus_obj.read_byte_data(addr, reg_msb)
    lsb = bus_obj.read_byte_data(addr, reg_msb + 1)
    return (msb << 8) + lsb

def setup_imu_for_repeat():
    global AC1, AC2, AC3, B1, B2, MB, MC, MD, AC4, AC5, AC6, bus, last_ahrs_update_time
    print("Repeat: Initializing IMU...")
    try: bus.write_byte_data(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00)
    except Exception as e: print(f"Repeat: Failed to wake MPU6050: {e}")
    try:
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70)
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 0xA0)
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00)
    except Exception as e: print(f"Repeat: Failed to configure HMC5883L: {e}")
    try:
        AC1 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xAA)
        AC2 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xAC)
        AC3 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xAE)
        AC4 = read_unsigned_word_big_endian(bus, BMP180_ADDR, 0xB0)
        AC5 = read_unsigned_word_big_endian(bus, BMP180_ADDR, 0xB2)
        AC6 = read_unsigned_word_big_endian(bus, BMP180_ADDR, 0xB4)
        B1 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xB6)
        B2 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xB8)
        MB = read_signed_word_big_endian(bus, BMP180_ADDR, 0xBA)
        MC = read_signed_word_big_endian(bus, BMP180_ADDR, 0xBC)
        MD = read_signed_word_big_endian(bus, BMP180_ADDR, 0xBE)
    except Exception as e: print(f"Repeat: Failed to read BMP180 calibration data: {e}")
    last_ahrs_update_time = time.monotonic() # Initialize for first dt calculation
    print("Repeat: IMU setup complete.")
    return True

def get_live_imu_data():
    # Your full, working get_imu_data function copied from teach.py
    imu_p = {
        "accel_x": 0, "accel_y": 0, "accel_z": 1.0, # Default to 1g down for stability
        "gyro_x": 0, "gyro_y": 0, "gyro_z": 0,
        "mag_x": 0.1, "mag_y": 0, "mag_z": 0, # Default non-zero mag to avoid issues if fusion uses it
        "temperature_mpu": 25, "pressure": 101325,
        "altitude_bmp": 0, "temperature_bmp": 25
    }
    try: # MPU6050
        imu_p["accel_x"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H) / 16384.0
        imu_p["accel_y"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H + 2) / 16384.0
        imu_p["accel_z"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H + 4) / 16384.0
        imu_p["gyro_x"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H) / 131.0
        imu_p["gyro_y"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 2) / 131.0
        imu_p["gyro_z"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 4) / 131.0
        temp_raw_mpu = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_TEMP_OUT_H)
        imu_p["temperature_mpu"] = temp_raw_mpu / 340.0 + 36.53
    except Exception as e_mpu: print(f"Repeat IMU Read Error MPU: {e_mpu}")

    try: # HMC5883L
        mag_x_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB)
        mag_z_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 2)
        mag_y_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 4)
        imu_p["mag_x"], imu_p["mag_y"], imu_p["mag_z"] = mag_x_raw * 0.92, mag_y_raw * 0.92, mag_z_raw * 0.92 # Example scale
    except Exception as e_hmc: print(f"Repeat IMU Read Error HMC: {e_hmc}")

    try: # BMP180
        bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP)
        time.sleep(0.005)
        UT = read_unsigned_word_big_endian(bus, BMP180_ADDR, BMP180_REG_RESULT_MSB)
        OSS = 0
        bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_PRESSURE_0 + (OSS << 6))
        time.sleep(0.015)
        UP_msb = bus.read_byte_data(BMP180_ADDR, 0xF6)
        UP_lsb = bus.read_byte_data(BMP180_ADDR, 0xF7)
        UP_xlsb = bus.read_byte_data(BMP180_ADDR, 0xF8)
        UP = ((UP_msb << 16) + (UP_lsb << 8) + UP_xlsb) >> (8 - OSS)

        # Đảm bảo các giá trị hiệu chuẩn không bằng không trước khi chia
        # Sử dụng // cho phép chia nguyên
        if AC5 == 0 or (X1_temp := (UT - AC6) * AC5 // (2**15)) + MD == 0:
            imu_p["temperature_bmp"], imu_p["pressure"], imu_p["altitude_bmp"] = 0, 0, 0
        else:
            X1 = X1_temp
            # Sử dụng // cho phép chia nguyên
            X2 = MC * (2**11) // (X1 + MD)
            B5 = X1 + X2
            # Nhiệt độ có thể giữ dạng float
            imu_p["temperature_bmp"] = (B5 + 8) / (2**4) / 10.0
            B6 = B5 - 4000
            # Sử dụng // cho phép chia nguyên
            X1 = (B2 * (B6 * B6 // (2**12))) // (2**11)
            X2 = AC2 * B6 // (2**11)
            X3 = X1 + X2
            # Sử dụng // cho phép chia nguyên. X3 bây giờ là số nguyên, << sẽ hoạt động.
            B3 = (((AC1 * 4 + X3) << OSS) + 2) // 4
            # Sử dụng // cho phép chia nguyên
            X1 = AC3 * B6 // (2**13)
            X2 = (B1 * (B6 * B6 // (2**12))) // (2**16)
            X3 = ((X1 + X2) + 2) // 4
            B4 = AC4 * (X3 + 32768) // (2**15)
            
            if B4 == 0: 
                imu_p["pressure"], imu_p["altitude_bmp"] = 0, 0
            else:
                B7 = (UP - B3) * (50000 >> OSS)
                # Phép chia ở đây nên là float để có áp suất chính xác
                p_calc = (B7 * 2) / B4 if B7 >= 0x80000000 else (B7 / B4) * 2
                
                # Sử dụng // cho các bước hiệu chỉnh áp suất
                X1_p = (p_calc / (2**8))**2
                X1_p = (X1_p * 3038) // (2**16)
                X2_p = (-7357 * p_calc) // (2**16)
                # Phép chia cuối cùng là float
                imu_p["pressure"] = p_calc + (X1_p + X2_p + 3791) / (2**4)
                
                if imu_p["pressure"] > 0:
                    imu_p["altitude_bmp"] = 44330 * (1.0 - pow(imu_p["pressure"] / 101325.0, 1/5.255))
                else: 
                    imu_p["altitude_bmp"] = 0

    except Exception as e:
        print(f"Lỗi khi đọc BMP180: {e}")
        # Có thể đặt giá trị mặc định ở đây nếu muốn
        imu_p["temperature_bmp"], imu_p["pressure"], imu_p["altitude_bmp"] = 0, 0, 0

    return imu_p

# --- Redis Publishing Function ---
def setup_redis_publisher():
    global redis_client_publisher
    try:
        print("Repeat: Connecting to Redis for publishing commands...")
        redis_client_publisher = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=False)
        redis_client_publisher.ping()
        print("Repeat: Redis client for publishing established.")
        return True
    except Exception as e:
        print(f"Repeat: Failed to connect Redis for publishing: {e}")
        redis_client_publisher = None
        return False

def publish_vehicle_commands_to_redis(left_dir, left_speed, right_dir, right_speed, s1_angle, s2_angle, e_stop=0):
    if redis_client_publisher:
        command_payload = {
            'motor_left_dir': int(left_dir), 'motor_left_speed': int(left_speed),
            'motor_right_dir': int(right_dir), 'motor_right_speed': int(right_speed),
            'servo1_angle': int(s1_angle), 'servo2_angle': int(s2_angle),
            'e_stop_active': int(e_stop),
            'timestamp': time.time() # Timestamp of command generation by repeat.py
        }
        try:
            redis_client_publisher.xadd(PROCESSED_COMMAND_STREAM_NAME, command_payload)
        except Exception as e:
            print(f"Repeat: Error publishing vehicle commands to Redis: {e}")

# --- PID Controller (Your SimplePID class from previous code) ---
class SimplePID:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(-100, 100), anti_windup_limit=50):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.prev_error, self.integral = 0, 0
        self.output_limits = output_limits
        self.anti_windup_limit = anti_windup_limit # Max integral value
        self.last_time = time.monotonic()

    def update(self, current_value, dt): # dt is now passed in
        if dt <= 1e-6: # Avoid division by zero or excessively small dt
            return 0.0 # Or previous output

        error = self.setpoint - current_value
        # Handle yaw wrap-around if current_value and setpoint are angles in degrees
        if abs(error) > 180.0: # Assuming angles are 0-360 or -180 to 180
            error -= math.copysign(360.0, error)

        self.integral += error * dt
        # Anti-windup for integral term
        self.integral = max(-self.anti_windup_limit, min(self.anti_windup_limit, self.integral))
        
        derivative = (error - self.prev_error) / dt
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        self.prev_error = error
        # self.last_time is not updated here as dt is passed in
        return max(self.output_limits[0], min(self.output_limits[1], output))

    def set_target(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0.0 # Reset integral when setpoint changes
        self.prev_error = 0.0 # Reset previous error
        # self.last_time = time.monotonic() # Reset time for PID internal dt if it were calculating it


# --- Main Repeat Logic ---
def main_repeat_phase(session_to_repeat_path, use_ips_from_log_flag):
    global ahrs_filter, last_ahrs_update_time, current_quaternion_ahrs # Thêm current_quaternion_ahrs

    log_file_path = os.path.join(session_to_repeat_path, "data.csv")
    if not os.path.exists(log_file_path):
        print(f"Repeat: Log file not found: {log_file_path}")
        return

    # Setup Redis for publishing commands, IMU for live data
    if not setup_redis_publisher(): return
    if not setup_imu_for_repeat(): return # Initializes IMU and last_ahrs_update_time

    # Initialize PID for Yaw correction (Tune Kp, Ki, Kd carefully!)
    yaw_pid = SimplePID(Kp=1.5, Ki=0.05, Kd=0.25, output_limits=(-100, 100), anti_windup_limit=50)

    print(f"Repeat: Starting Repeat phase for session: {session_to_repeat_path}")
    
    recorded_data = []
    try:
        with open(log_file_path, mode='r', newline='') as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                print(f"Repeat: CSV file '{log_file_path}' is empty or has no headers.")
                return
            for row in reader:
                recorded_data.append(row) # Store rows as dicts
    except Exception as e:
        print(f"Repeat: Error reading CSV log file: {e}")
        return
    
    if not recorded_data:
        print("Repeat: No data found in log file.")
        return

    start_time_real = time.monotonic() # Overall start time of repeat phase
    start_time_log = float(recorded_data[0]['timestamp']) # Timestamp of the first log entry

    # Attempt to get an initial absolute yaw if possible, otherwise start with 0 or first fused reading
    # This is crucial for the PID's absolute target.
    # For now, we'll let Madgwick converge from an arbitrary start.

    for i, log_entry_str_dict in enumerate(recorded_data):
        try:
            # Convert string values from CSV to appropriate types
            log_entry = {}
            for key, value_str in log_entry_str_dict.items():
                if value_str is None or value_str == '':
                    log_entry[key] = None # Handle empty strings as None
                elif key in ['timestamp', 'accel_x', 'accel_y', 'accel_z', 
                             'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z',
                             'temp_mpu', 'temp_bmp', 'pressure', 'altitude_bmp',
                             'ips_x', 'ips_y', 'ips_yaw', 'ips_quality','fused_yaw_deg']:
                    try:
                        log_entry[key] = float(value_str) # Chuyển đổi giá trị chuỗi (value_str) sang kiểu số thực (float)
                    except ValueError:
                        print(f"Repeat: ValueError converting '{value_str}' to float for key '{key}'. Setting to None.")
                        log_entry[key] = None # Nếu không chuyển đổi được (ví dụ value_str không phải là số), gán là None
                
                elif key in ['motor_left_dir', 'motor_left_speed', 'motor_right_dir', 'motor_right_speed',
                             'servo1_angle', 'servo2_angle', 'e_stop_active']:
                    try:
                        log_entry[key] = int(value_str) # Chuyển đổi giá trị chuỗi (value_str) sang kiểu số nguyên (int)
                    except ValueError:
                        print(f"Repeat: ValueError converting '{value_str}' to int for key '{key}'. Setting to None.")
                        log_entry[key] = None # Nếu không chuyển đổi được, gán là None
                
                else:
                    log_entry[key] = value_str # Keep frame_file as string

            log_timestamp = log_entry['timestamp']
            
            # --- Target commands from log ---
            log_m_left_dir = log_entry.get('motor_left_dir', 0)
            log_m_left_speed = log_entry.get('motor_left_speed', 0)
            log_m_right_dir = log_entry.get('motor_right_dir', 0)
            log_m_right_speed = log_entry.get('motor_right_speed', 0)
            log_s1_angle = log_entry.get('servo1_angle', SERVO_CENTER_ANGLE)
            log_s2_angle = log_entry.get('servo2_angle', SERVO_CENTER_ANGLE)

            # --- Target Yaw from log (IMPORTANT: How do you define this?) ---
            # Option A: If you logged an absolute fused yaw during teach:

            target_absolute_yaw = log_entry.get('fused_yaw_deg', None) 
            # Option B: If you want to replicate yaw rate changes:

            target_yaw_rate_from_log = log_entry.get('gyro_z', 0.0) # In deg/s

            # --- Wait to match log timing ---
            current_log_time_offset = log_timestamp - start_time_log
            while (time.monotonic() - start_time_real) < current_log_time_offset:
                time.sleep(0.0005) # Sleep very briefly, but actively wait

            # --- Get Live Sensor Data & Update AHRS ---
            current_monotonic_time = time.monotonic()
            dt = current_monotonic_time - last_ahrs_update_time
            if dt <= 1e-6: dt = SAMPLE_PERIOD # Ensure positive dt for AHRS update
            last_ahrs_update_time = current_monotonic_time
            
            live_imu = get_live_imu_data()
            gyro_np = np.array([
                math.radians(live_imu.get('gyro_x',0.0)),
                math.radians(live_imu.get('gyro_y',0.0)),
                math.radians(live_imu.get('gyro_z',0.0))
            ])

            accel_np = np.array([
                live_imu.get('accel_x',0.0),
                live_imu.get('accel_y',0.0),
                live_imu.get('accel_z',1.0)
            ])

            mag_np = np.array([
                live_imu.get('mag_x',0.1),
                live_imu.get('mag_y',0.0),
                live_imu.get('mag_z',0.0)
            ])

            # Cập nhật bộ lọc Madgwick
            # Dựa trên ví dụ bạn cung cấp: madgwick.updateMARG(Q[t-1], gyr=..., acc=..., mag=...)
            try:
                # Nếu Madgwick được khởi tạo với sample_period, nó có thể không cần dt ở đây.
                # Nếu không, nó sẽ cần dt.
                # Tên tham số cho quaternion cũ có thể là 'q', 'Q', hoặc không có tên (tham số vị trí).
                # Giả sử là tham số vị trí đầu tiên như trong ví dụ `Q[t-1]` của bạn.
                
                # Kiểm tra xem hàm updateMARG có nhận 'dt' không.
                # Đây là cách kiểm tra hơi phức tạp, cách đơn giản là thử và bắt TypeError
                # has_dt_param = 'dt' in ahrs_filter.updateMARG.__code__.co_varnames
                
                # new_q_np = ahrs_filter.updateMARG(current_quaternion_ahrs, gyr=gyro_np, acc=accel_np, mag=mag_np, dt=dt)
                # Hoặc nếu thư viện dùng sample_period đã set và không cần dt ở update:
                new_q_np = ahrs_filter.updateMARG(current_quaternion_ahrs, gyr=gyro_np, acc=accel_np, mag=mag_np)

                if new_q_np is not None and isinstance(new_q_np, np.ndarray) and new_q_np.shape == (4,):
                    current_quaternion_ahrs = new_q_np
                # Một số thư viện có thể cập nhật thuộc tính .Q nội bộ thay vì trả về
                elif hasattr(ahrs_filter, 'Q') and isinstance(ahrs_filter.Q, np.ndarray) and ahrs_filter.Q.shape == (4,):
                    current_quaternion_ahrs = ahrs_filter.Q
                # else:
                    # print("Repeat: Madgwick updateMARG did not return a valid quaternion and .Q is not as expected.")

            except TypeError as te:
                # Nếu lỗi TypeError, có thể do thiếu/thừa tham số dt
                print(f"Repeat: TypeError calling updateMARG. Trying with/without dt or checking method signature. Error: {te}")
                # Ví dụ, thử gọi với dt nếu trước đó không có, hoặc ngược lại
                try:
                    # Giả sử nếu lỗi là do thiếu dt, thư viện sẽ có tham số dt
                    new_q_np = ahrs_filter.updateMARG(current_quaternion_ahrs, gyr=gyro_np, acc=accel_np, mag=mag_np, dt=dt)
                    if new_q_np is not None: current_quaternion_ahrs = new_q_np
                except Exception as e_fallback:
                    print(f"Repeat: Fallback updateMARG call also failed: {e_fallback}")
            except Exception as e_ahrs:
                print(f"Repeat: Error updating AHRS: {e_ahrs}")
            # (Giữ nguyên phần chuyển đổi quaternion sang Euler bạn đã có)
        
            # Chuyển quaternion (NumPy array) sang góc Euler
            q0, q1, q2, q3 = current_quaternion_ahrs[0], current_quaternion_ahrs[1], current_quaternion_ahrs[2], current_quaternion_ahrs[3]

            roll_rad = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))
            pitch_rad = math.asin(max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1))))
            yaw_rad = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))

            siny_cosp = 2 * (q0 * q3 + q1 * q2)
            cosy_cosp = 1 - 2 * (q2**2 + q3**2)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            live_fused_yaw_deg = math.degrees(yaw_rad)
            if live_fused_yaw_deg < 0: live_fused_yaw_deg += 360
            



            # --- PID Correction for Yaw ---
            # **IMPORTANT**: Decide your PID strategy here.
            # Strategy 1: Control based on absolute yaw. Requires a target_absolute_yaw from log.
            # This 'absolute_yaw_log' column needs to be created during 'teach.py' by running Madgwick there too.
            target_absolute_yaw = float(log_entry.get('absolute_yaw_log', live_fused_yaw_deg)) # Fallback
            yaw_pid.set_target(target_absolute_yaw)
            yaw_correction = yaw_pid.update(live_fused_yaw_deg, dt)

            # Strategy 2: Control based on yaw rate (simpler if absolute yaw is hard to get from log)
            live_gyro_z_dps = live_imu.get('gyro_z', 0.0)
            yaw_pid.set_target(target_yaw_rate_from_log) # target_yaw_rate_from_log = log_entry.get('gyro_z',0.0)
            yaw_correction = yaw_pid.update(live_gyro_z_dps, dt)


            # Apply correction to motor speeds
            final_m_left_speed = float(log_m_left_speed)
            final_m_right_speed = float(log_m_right_speed)

            # Positive yaw_correction from PID means need to increase yaw (turn right if yaw increases clockwise)
            # To turn right: left motor faster, right motor slower (or backward)
            final_m_left_speed += yaw_correction  
            final_m_right_speed -= yaw_correction
            
            final_m_left_speed = max(0, min(255, int(final_m_left_speed)))
            final_m_right_speed = max(0, min(255, int(final_m_right_speed)))

            final_m_left_dir = log_m_left_dir if final_m_left_speed > 0 else 0
            final_m_right_dir = log_m_right_dir if final_m_right_speed > 0 else 0
            
            # --- Publish Commands to Redis ---
            publish_vehicle_commands_to_redis(
                final_m_left_dir, final_m_left_speed,
                final_m_right_dir, final_m_right_speed,
                log_s1_angle, log_s2_angle
            )

            if (i + 1) % 10 == 0:
                print(f"Repeat [{i+1}/{len(recorded_data)}]: TargetYaw={yaw_pid.setpoint:.1f}, LiveYaw={live_fused_yaw_deg:.1f}, Corr={yaw_correction:.1f} | "
                      f"LSpd={final_m_left_speed}, RSpd={final_m_right_speed}")

        except KeyboardInterrupt:
            print("\nRepeat: Phase interrupted by user.")
            break
        except ValueError as e:
            print(f"Repeat: Skipping row {i} due to data conversion error: {log_entry_str_dict} - {e}")
        except Exception as e:
            print(f"Repeat: Error in loop for row {i} (log_ts: {log_entry_str_dict.get('timestamp', 'N/A')}): {e}")
            # import traceback # For more detailed error
            # traceback.print_exc()
            # break # Consider breaking on critical errors

    print("Repeat: Repeat phase finished.")
    publish_vehicle_commands_to_redis(0,0,0,0, SERVO_CENTER_ANGLE, SERVO_CENTER_ANGLE, e_stop=1)


if __name__ == "__main__":
    dataset_base_dir = "T_R_navigation_method/dataset_teach" # Your teach data directory
    session_to_run = None
    
    if not setup_imu_for_repeat():
        print("Repeat: Failed to initialize IMU. Exiting.")
        sys.exit(1)

    # Determine which session to repeat
    if len(sys.argv) > 1:
        potential_session_path = sys.argv[1]
        if os.path.isdir(potential_session_path) and os.path.exists(os.path.join(potential_session_path, "data.csv")):
            session_to_run = potential_session_path
            print(f"Repeat: Will use specified session: {session_to_run}")
        else:
            print(f"Repeat: Specified path '{potential_session_path}' is not a valid session directory. Trying latest.")
    
    if not session_to_run:
        if not os.path.isdir(dataset_base_dir):
            print(f"Repeat: Dataset directory '{dataset_base_dir}' not found.")
            sys.exit(1)
        all_sessions = [d for d in os.listdir(dataset_base_dir) if os.path.isdir(os.path.join(dataset_base_dir, d))]
        if not all_sessions:
            print(f"Repeat: No sessions found in '{dataset_base_dir}'. Run teach phase first.")
            sys.exit(1)
        latest_session_name = sorted(all_sessions)[-1] # Sorts alphabetically, usually works for timestamped names
        session_to_run = os.path.join(dataset_base_dir, latest_session_name)
        print(f"Repeat: Will use latest session: {session_to_run}")
    
    use_ips_if_available_in_log = False # Not actively used for correction in this version

    if session_to_run:
        main_repeat_phase(session_to_run, use_ips_if_available_in_log)
    else:
        print("Repeat: No session specified or found to repeat.")

    if redis_client_publisher:
        redis_client_publisher.close()
        print("Repeat: Redis publishing client closed.")
    print("Repeat: Script finished.")
    sys.exit(0)