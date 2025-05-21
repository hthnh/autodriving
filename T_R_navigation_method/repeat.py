# repeat.py
# Reads a taught session, applies IMU-based correction, 
# and publishes final commands to Redis for uart_sender.py

import os
import time
import csv
import smbus # For I2C with IMU
import redis # For publishing commands
import math
import sys   # For command line arguments and exit

# --- Configuration ---

SERVO_CENTER_ANGLE = 90



# Redis Configuration (for publishing processed commands)
REDIS_HOST = 'localhost'
REDIS_PORT = 6379
redis_client_publisher = None # Separate client instance for publishing
PROCESSED_COMMAND_STREAM_NAME = 'vehicle:commands_processed' # Stream to publish to
# IMU GY-87 Global Variables & Constants
# (Copied from your provided code - ensure these are complete and correct)
bus = smbus.SMBus(1)
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

# BMP180 Calibration data (global or passed around)
AC1, AC2, AC3, B1, B2, MB, MC, MD = 0,0,0,0,0,0,0,0
AC4, AC5, AC6 = 0,0,0

# Global for simple yaw estimation (REPLACE WITH ROBUST SENSOR FUSION)
current_estimated_yaw = 0.0
last_fusion_time = 0.0

# --- IMU Helper Functions (Copied from your code) ---
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

def setup_imu_for_repeat(): # Renamed to avoid confusion if imported
    global AC1, AC2, AC3, B1, B2, MB, MC, MD, AC4, AC5, AC6, bus
    print("Repeat: Initializing IMU...")
    # Wake MPU6050
    try: bus.write_byte_data(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00)
    except Exception as e: print(f"Repeat: Failed to wake MPU6050: {e}")
    # Configure HMC5883L
    try:
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70) # 8-sample avg, 15Hz
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 0xA0) # Gain (adjust from datasheet)
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00)    # Continuous measurement
    except Exception as e: print(f"Repeat: Failed to configure HMC5883L: {e}")
    # Read BMP180 calibration data
    try:
        AC1 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xAA)
        AC2 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xAC)
        # ... (read all other BMP180 calibration registers: AC3-AC6, B1, B2, MB, MC, MD) ...
        AC3 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xAE)
        AC4 = read_unsigned_word_big_endian(bus, BMP180_ADDR, 0xB0)
        AC5 = read_unsigned_word_big_endian(bus, BMP180_ADDR, 0xB2)
        AC6 = read_unsigned_word_big_endian(bus, BMP180_ADDR, 0xB4)
        B1 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xB6)
        B2 = read_signed_word_big_endian(bus, BMP180_ADDR, 0xB8)
        MB = read_signed_word_big_endian(bus, BMP180_ADDR, 0xBA)
        MC = read_signed_word_big_endian(bus, BMP180_ADDR, 0xBC)
        MD = read_signed_word_big_endian(bus, BMP180_ADDR, 0xBE)
        # print("Repeat: IMU BMP180 calibration data read.") # Less verbose
    except Exception as e: print(f"Repeat: Failed to read BMP180 calibration data: {e}")
    print("Repeat: IMU setup complete.")
    return True


def get_live_imu_data():
    # Ensure this function correctly reads all sensors from GY-87
    # and returns a dictionary with the same keys as logged by teach.py
    # This is your full get_imu_data function
    imu_p = { # Using a different name to avoid conflict if you had 'imu_payload' global
        "accel_x": 0, "accel_y": 0, "accel_z": 0, "gyro_x": 0, "gyro_y": 0, "gyro_z": 0,
        "mag_x": 0, "mag_y": 0, "mag_z": 0, "temperature_mpu": 0, "pressure": 0,
        "altitude_bmp": 0, "temperature_bmp": 0
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
    except Exception: pass # Silently fail for brevity in example

    try: # HMC5883L (X, Z, Y order from registers)
        mag_x_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB)
        mag_z_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 2)
        mag_y_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 4)
        # Note: Axis and scaling need calibration for your specific setup and HMC5883L orientation
        imu_p["mag_x"], imu_p["mag_y"], imu_p["mag_z"] = mag_x_raw * 0.92, mag_y_raw * 0.92, mag_z_raw * 0.92
    except Exception: pass

    try: # BMP180
        bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP)
        time.sleep(0.005) # Wait for measurement
        UT = read_unsigned_word_big_endian(bus, BMP180_ADDR, BMP180_REG_RESULT_MSB)
        OSS = 0 # Oversampling setting
        bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_PRESSURE_0 + (OSS << 6))
        time.sleep(0.005 + (0.003 * (1<<OSS)))
        UP_msb = bus.read_byte_data(BMP180_ADDR, 0xF6)
        UP_lsb = bus.read_byte_data(BMP180_ADDR, 0xF7)
        UP_xlsb = bus.read_byte_data(BMP180_ADDR, 0xF8)
        UP = ((UP_msb << 16) + (UP_lsb << 8) + UP_xlsb) >> (8 - OSS)
        # Calculations (ensure AC1-MD are correctly read in setup_imu_for_repeat)
        if AC5 == 0 or (X1_temp := (UT - AC6) * AC5 / (2**15)) + MD == 0 : # Check for division by zero
             # Handle error or set default values if calibration data is missing
            imu_p["temperature_bmp"] = 0
            imu_p["pressure"] = 0
            imu_p["altitude_bmp"] = 0
        else:
            X1 = X1_temp
            X2 = MC * (2**11) / (X1 + MD)
            B5 = X1 + X2
            imu_p["temperature_bmp"] = (B5 + 8) / (2**4) / 10.0 # Temp in Celsius
            B6 = B5 - 4000
            X1 = (B2 * (B6 * B6 / (2**12))) / (2**11)
            X2 = AC2 * B6 / (2**11)
            X3 = X1 + X2
            B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4
            X1 = AC3 * B6 / (2**13)
            X2 = (B1 * (B6 * B6 / (2**12))) / (2**16)
            X3 = ((X1 + X2) + 2) / (2**2)
            B4 = AC4 * (X3 + 32768) / (2**15)
            if B4 == 0: # Avoid division by zero for pressure calculation
                imu_p["pressure"] = 0
                imu_p["altitude_bmp"] = 0
            else:
                B7 = (UP - B3) * (50000 >> OSS)
                p_calc = (B7 * 2) / B4 if B7 >= 0x80000000 else (B7 / B4) * 2
                X1 = (p_calc / (2**8))**2
                X1 = (X1 * 3038) / (2**16)
                X2 = (-7357 * p_calc) / (2**16)
                imu_p["pressure"] = p_calc + (X1 + X2 + 3791) / (2**4) # Pressure in Pa
                if imu_p["pressure"] > 0:
                    imu_p["altitude_bmp"] = 44330 * (1.0 - pow(imu_p["pressure"] / 101325.0, 1/5.255))
                else:
                    imu_p["altitude_bmp"] = 0
    except Exception: pass
    return imu_p

# --- Redis Publishing Function ---
def setup_redis_publisher():
    global redis_client_publisher
    try:
        print("Repeat: Connecting to Redis for publishing commands...")
        redis_client_publisher = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
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
            'timestamp': time.time()
        }
        try:
            # print(f"Repeat: Publishing to {PROCESSED_COMMAND_STREAM_NAME}: {command_payload}") # Debug
            redis_client_publisher.xadd(PROCESSED_COMMAND_STREAM_NAME, command_payload)
        except Exception as e:
            print(f"Repeat: Error publishing vehicle commands to Redis: {e}")

# --- Sensor Fusion & PID (Placeholders - requires significant development) ---
def update_fused_orientation(live_imu_data, dt):
    global current_estimated_yaw, last_fusion_time # Using global for simplicity in example
    # This is a VERY basic yaw estimation using gyro integration.
    # YOU NEED TO REPLACE THIS WITH A ROBUST SENSOR FUSION ALGORITHM
    # (e.g., Madgwick, Mahony, EKF, or at least a Complementary Filter for roll/pitch/yaw)
    
    # If last_fusion_time is not set, initialize it
    if last_fusion_time == 0.0:
        last_fusion_time = time.monotonic() - dt # Estimate previous time for first dt
        
    gyro_z_dps = live_imu_data.get('gyro_z', 0.0)
    
    # Simple yaw integration (prone to drift)
    current_estimated_yaw += gyro_z_dps * dt
    current_estimated_yaw %= 360
    if current_estimated_yaw < 0: current_estimated_yaw += 360
    
    # Placeholder for roll and pitch
    # roll = math.atan2(live_imu_data.get('accel_y', 0), live_imu_data.get('accel_z', 1)) * 180 / math.pi
    # pitch = math.atan2(-live_imu_data.get('accel_x', 0), 
    #                    math.sqrt(live_imu_data.get('accel_y', 0)**2 + live_imu_data.get('accel_z', 1)**2)) * 180 / math.pi
    
    return {"yaw": current_estimated_yaw, "pitch": 0, "roll": 0} # Return a dictionary

class SimplePID:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(-100, 100), anti_windup_limit=50):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.prev_error, self.integral = 0, 0
        self.output_limits = output_limits
        self.anti_windup_limit = anti_windup_limit
        self.last_time = time.monotonic()

    def update(self, current_value, dt):
        if dt <= 0: dt = 1e-3 # avoid division by zero or negative dt

        error = self.setpoint - current_value
        if abs(error) > 180: # Handle yaw wrap-around (for angles in degrees)
            error -= math.copysign(360, error)

        self.integral += error * dt
        # Anti-windup
        self.integral = max(-self.anti_windup_limit, min(self.anti_windup_limit, self.integral))

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        self.last_time = time.monotonic() # Update time for next dt calculation
        return max(self.output_limits[0], min(self.output_limits[1], output))

    def set_target(self, setpoint):
        self.setpoint = setpoint
        # Reset integral and previous error when setpoint changes significantly or on init
        self.integral = 0
        # self.prev_error = 0 # Or calculate initial error to new setpoint
        self.last_time = time.monotonic()


# --- Main Repeat Logic ---
def main_repeat_phase(session_to_repeat_path, use_ips_from_log_flag): # Added flag
    global current_estimated_yaw, last_fusion_time # For simple yaw tracking

    log_file_path = os.path.join(session_to_repeat_path, "data.csv")
    if not os.path.exists(log_file_path):
        print(f"Repeat: Log file not found: {log_file_path}")
        return

    if not setup_redis_publisher(): return # For sending commands to uart_sender
    if not setup_imu_for_repeat(): return

    # PID for Yaw correction (Tune these gains carefully!)
    # Output range could be +/- motor speed adjustment
    yaw_pid = SimplePID(Kp=1.2, Ki=0.1, Kd=0.15, output_limits=(-80, 80), anti_windup_limit=40)

    print(f"Repeat: Starting Repeat phase for session: {session_to_repeat_path}")
    
    recorded_data = []
    try:
        with open(log_file_path, mode='r', newline='') as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                print(f"Repeat: CSV file '{log_file_path}' is empty or has no headers.")
                return
            for row in reader:
                recorded_data.append(row)
    except Exception as e:
        print(f"Repeat: Error reading CSV log file: {e}")
        return
    
    if not recorded_data:
        print("Repeat: No data found in log file.")
        return

    start_time_real = time.monotonic()
    start_time_log = float(recorded_data[0]['timestamp'])
    
    # Initialize current_estimated_yaw from the first valid IMU reading or magnetometer
    # This part needs a robust way to get initial absolute yaw if possible.
    # For now, it will start at 0 and integrate.
    initial_live_imu = get_live_imu_data()
    last_fusion_time = time.monotonic() # Initialize for first dt calculation
    # You might want a short delay here to get a couple of IMU readings for better initial orientation
    # current_estimated_yaw = calculate_initial_yaw(initial_live_imu) # Implement this

    previous_loop_real_time = start_time_real

    for i, row_str_dict in enumerate(recorded_data):
        try:
            # Convert relevant log data from string to float/int
            row = {key: float(value) if value not in (None, '') else 0.0 for key, value in row_str_dict.items() 
                   if key not in ['frame_file']} # Keep frame_file as string
            row['frame_file'] = row_str_dict.get('frame_file', '')
            # Ensure directions are int
            for key in ['motor_left_dir', 'motor_right_dir', 'e_stop_active']:
                 row[key] = int(row[key]) if row.get(key) is not None else 0
            for key in ['motor_left_speed', 'motor_right_speed', 'servo1_angle', 'servo2_angle']:
                 row[key] = int(row[key]) if row.get(key) is not None else 0


            log_timestamp = float(row_str_dict['timestamp']) # Use original string for precision
            
            # Target commands from log
            log_m_left_dir = row['motor_left_dir']
            log_m_left_speed = row['motor_left_speed']
            log_m_right_dir = row['motor_right_dir']
            log_m_right_speed = row['motor_right_speed']
            log_s1_angle = row['servo1_angle']
            log_s2_angle = row['servo2_angle']

            # Target IMU data from log for PID setpoint (e.g., logged yaw_rate or absolute_yaw)
            # If you logged absolute yaw (e.g., "log_abs_yaw") from Teach phase:
            # target_yaw_for_pid = row.get('log_abs_yaw', current_estimated_yaw) # Fallback to current
            # OR, if you want to follow yaw_rate:
            target_yaw_rate_for_pid = row.get('gyro_z', 0.0)

            # --- Wait to match log timing ---
            current_log_time_offset = log_timestamp - start_time_log
            
            # Wait until it's time to execute this log entry
            while (time.monotonic() - start_time_real) < current_log_time_offset:
                time.sleep(0.001) # Sleep very briefly while waiting

            # --- Get Live Sensor Data & Sensor Fusion ---
            current_real_time = time.monotonic()
            dt = current_real_time - last_fusion_time
            if dt <= 0.001: # Ensure dt is not too small or zero
                dt = 0.001 
            last_fusion_time = current_real_time
            
            live_imu = get_live_imu_data()
            fused_orientation = update_fused_orientation(live_imu, dt) # This updates global current_estimated_yaw
            live_fused_yaw = fused_orientation["yaw"] # Get the updated yaw
            live_gyro_z = live_imu.get('gyro_z', 0.0)

            # --- PID Correction for Yaw ---
            # Option 1: PID tries to make live yaw_rate match logged yaw_rate
            yaw_pid.set_target(target_yaw_rate_for_pid)
            yaw_correction = yaw_pid.update(live_gyro_z, dt)

            # Option 2: PID tries to make live absolute_yaw match a logged absolute_yaw
            # (This requires a reliable absolute_yaw in your log and from live fusion)
            # target_absolute_yaw_from_log = row.get('absolute_yaw_column_name', live_fused_yaw) # Default to current if not in log
            # yaw_pid.set_target(target_absolute_yaw_from_log)
            # yaw_correction = yaw_pid.update(live_fused_yaw, dt)


            # Apply correction to motor speeds
            # This logic assumes yaw_correction is a value to add/subtract from differential speed
            # Positive yaw_correction might mean "need to turn more left" if setpoint is a smaller yaw rate
            # or "need to turn more right" if setpoint is a larger yaw rate.
            # This part is highly dependent on your PID output meaning and vehicle kinematics.
            # Let's assume positive yaw_correction means "turn right" relative to current trajectory.
            
            # Base speeds from log
            final_m_left_speed = float(log_m_left_speed)
            final_m_right_speed = float(log_m_right_speed)

            # Apply correction: if correction is positive, turn right (increase left, decrease right for forward motion)
            final_m_left_speed += yaw_correction
            final_m_right_speed -= yaw_correction
            
            # Clamp speeds
            final_m_left_speed = max(0, min(255, int(final_m_left_speed)))
            final_m_right_speed = max(0, min(255, int(final_m_right_speed)))

            final_m_left_dir = log_m_left_dir if final_m_left_speed > 0 else 0
            final_m_right_dir = log_m_right_dir if final_m_right_speed > 0 else 0
            
            # --- Publish Commands to Redis for uart_sender.py ---
            publish_vehicle_commands_to_redis(
                final_m_left_dir, final_m_left_speed,
                final_m_right_dir, final_m_right_speed,
                log_s1_angle, log_s2_angle
            )

            if (i + 1) % 10 == 0:
                print(f"Repeat [{i+1}/{len(recorded_data)}]: LogGyroZ={target_yaw_rate_for_pid:.2f}, LiveGyroZ={live_gyro_z:.2f}, Corr={yaw_correction:.2f} | "
                      f"LSpd={final_m_left_speed}, RSpd={final_m_right_speed} | LiveYawEst={live_fused_yaw:.1f}")

        except KeyboardInterrupt:
            print("\nRepeat: Repeat phase interrupted by user.")
            break
        except ValueError as e: # Catch errors converting string data from CSV to float/int
            print(f"Repeat: Skipping row {i} due to data conversion error: {row_str_dict} - {e}")
        except Exception as e:
            print(f"Repeat: Error during repeat loop for row {i} (log_ts: {row_str_dict.get('timestamp', 'N/A')}): {e}")
            # import traceback
            # traceback.print_exc() # For detailed error
            # break # Consider breaking on critical errors

    print("Repeat: Repeat phase finished.")
    # Send final stop command via Redis
    publish_vehicle_commands_to_redis(0,0,0,0, SERVO_CENTER_ANGLE, SERVO_CENTER_ANGLE, e_stop=1)


if __name__ == "__main__":
    # --- Crucial: Ensure IMU functions from teach.py are defined or imported in this scope ---
    # For this example, they are defined globally in this file.
    
    dataset_base_dir = "T_R_navigation_method/dataset_teach" 
    session_to_run = None
    
    if not setup_imu_for_repeat(): # Call IMU setup once
        print("Repeat: Failed to initialize IMU. Exiting.")
        sys.exit(1)

    if len(sys.argv) > 1:
        potential_session_path = sys.argv[1]
        if os.path.isdir(potential_session_path) and os.path.exists(os.path.join(potential_session_path, "data.csv")):
            session_to_run = potential_session_path
        else:
            print(f"Repeat: Specified path '{potential_session_path}' is not a valid session directory.")
    
    if not session_to_run:
        if not os.path.isdir(dataset_base_dir):
            print(f"Repeat: Dataset directory '{dataset_base_dir}' not found.")
            sys.exit(1)
        all_sessions = [d for d in os.listdir(dataset_base_dir) if os.path.isdir(os.path.join(dataset_base_dir, d))]
        if not all_sessions:
            print(f"Repeat: No sessions found in '{dataset_base_dir}'. Run teach phase first.")
            sys.exit(1)
        latest_session_name = sorted(all_sessions)[-1]
        session_to_run = os.path.join(dataset_base_dir, latest_session_name)

    # For this script, IPS data is read from the log if present,
    # but active correction using live IPS is not implemented in this version.
    # The 'use_ips_from_log_flag' could control whether logged IPS data is used as a target.
    use_ips_if_available_in_log = False # You can change this or make it a cmd arg

    if session_to_run:
        main_repeat_phase(session_to_run, use_ips_if_available_in_log)
    else:
        print("Repeat: No session specified or found to repeat.")

    if redis_client_publisher: # Close Redis client used for publishing
        redis_client_publisher.close()
        print("Repeat: Redis publishing client closed.")
    print("Repeat: Script finished.")
    sys.exit(0)