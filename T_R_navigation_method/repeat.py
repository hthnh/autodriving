# repeat.py
import os
import time
import csv
import smbus # For I2C
import serial # For UART to Arduino Master
import math
# Bạn có thể cần các thư viện cho bộ lọc IMU (ví dụ: Madgwick, Mahony)
# hoặc tự viết một bộ lọc đơn giản (Complementary filter).

# ---------- UART to Arduino Master SETUP ----------
SERIAL_PORT_MASTER = '/dev/serial0' # Hoặc cổng UART của bạn
BAUD_RATE_MASTER = 9600
ser_master = None

# ---------- IPC SETUP (Unix Domain Socket) ----------
SOCKET_PATH = "/tmp/robot_control_socket" # Path for the socket file

# ---------- IMU GY-87 SETUP ----------
bus = smbus.SMBus(1) # I2C bus 1 on Raspberry Pi

# MPU6050 (Accelerometer and Gyroscope)
MPU6050_ADDR = 0x68 # Standard address
MPU6050_REG_PWR_MGMT_1 = 0x6B
MPU6050_REG_ACCEL_XOUT_H = 0x3B
MPU6050_REG_GYRO_XOUT_H = 0x43
MPU6050_REG_TEMP_OUT_H = 0x41

# HMC5883L (Magnetometer) - Part of GY-87
HMC5883L_ADDR = 0x1E # Standard address
HMC5883L_REG_CONFIG_A = 0x00
HMC5883L_REG_CONFIG_B = 0x01
HMC5883L_REG_MODE = 0x02
HMC5883L_REG_DATA_X_MSB = 0x03
# You might need to calibrate the magnetometer and adjust gain/scale

# BMP180 (Barometer/Temperature) - Part of GY-87
BMP180_ADDR = 0x77 # Standard address
# BMP180 calibration registers (read once during setup)
AC1_H = 0xAA, AC2_H = 0xAC, ..., B2_H = 0xB8, MB_H = 0xBA, MC_H = 0xBC, MD_H = 0xBE
# BMP180 control and data registers
BMP180_REG_CONTROL = 0xF4
BMP180_REG_RESULT_MSB = 0xF6
BMP180_CMD_READ_TEMP = 0x2E
BMP180_CMD_READ_PRESSURE_0 = 0x34 # Oversampling 0


# --- BMP180 Calibration Data (variables to store them) ---
# These will be read in setup_imu()
AC1, AC2, AC3, B1, B2, MB, MC, MD = 0,0,0,0,0,0,0,0
VB1, VB2 = 0,0 # For virtual sensor, not directly from BMP180 registers but used in calculation
AC4, AC5, AC6 = 0,0,0

def setup_imu():
    global AC1, AC2, AC3, B1, B2, MB, MC, MD, AC4, AC5, AC6
    # Wake MPU6050
    try:
        bus.write_byte_data(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00)
    except Exception as e:
        print(f"Failed to wake MPU6050: {e}")

    # Configure HMC5883L (Example: 8-sample average, 15Hz data output, normal mode)
    try:
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70) # 0b01110000
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 0xA0) # Gain (adjust if needed)
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00) # Continuous measurement
    except Exception as e:
        print(f"Failed to configure HMC5883L: {e}")

    # Read BMP180 calibration data
    try:
        AC1 = read_signed_word_big_endian(BMP180_ADDR, 0xAA)
        AC2 = read_signed_word_big_endian(BMP180_ADDR, 0xAC)
        AC3 = read_signed_word_big_endian(BMP180_ADDR, 0xAE)
        AC4 = read_unsigned_word_big_endian(BMP180_ADDR, 0xB0)
        AC5 = read_unsigned_word_big_endian(BMP180_ADDR, 0xB2)
        AC6 = read_unsigned_word_big_endian(BMP180_ADDR, 0xB4)
        B1 = read_signed_word_big_endian(BMP180_ADDR, 0xB6)
        B2 = read_signed_word_big_endian(BMP180_ADDR, 0xB8)
        MB = read_signed_word_big_endian(BMP180_ADDR, 0xBA)
        MC = read_signed_word_big_endian(BMP180_ADDR, 0xBC)
        MD = read_signed_word_big_endian(BMP180_ADDR, 0xBE)
    except Exception as e:
        print(f"Failed to read BMP180 calibration data: {e}")

def read_signed_word(bus_obj, addr, reg):
    high = bus_obj.read_byte_data(addr, reg)
    low = bus_obj.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    return value - 65536 if value >= 0x8000 else value

def read_unsigned_word(bus_obj, addr, reg):
    high = bus_obj.read_byte_data(addr, reg)
    low = bus_obj.read_byte_data(addr, reg + 1)
    return (high << 8) + low

def read_signed_word_big_endian(bus_obj, addr, reg_msb): # For BMP180 calib
    msb = bus_obj.read_byte_data(addr, reg_msb)
    lsb = bus_obj.read_byte_data(addr, reg_msb + 1)
    val = (msb << 8) + lsb
    return val - 65536 if val >= 0x8000 else val

def read_unsigned_word_big_endian(bus_obj, addr, reg_msb): # For BMP180 calib
    msb = bus_obj.read_byte_data(addr, reg_msb)
    lsb = bus_obj.read_byte_data(addr, reg_msb + 1)
    return (msb << 8) + lsb


def get_imu_data():
    imu_payload = {
        "accel_x": 0, "accel_y": 0, "accel_z": 0,
        "gyro_x": 0, "gyro_y": 0, "gyro_z": 0,
        "mag_x": 0, "mag_y": 0, "mag_z": 0,
        "temperature_mpu": 0, "pressure": 0, "altitude_bmp": 0, "temperature_bmp": 0
    }
    try: # MPU6050
        imu_payload["accel_x"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H) / 16384.0
        imu_payload["accel_y"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H + 2) / 16384.0
        imu_payload["accel_z"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H + 4) / 16384.0
        imu_payload["gyro_x"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H) / 131.0
        imu_payload["gyro_y"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 2) / 131.0
        imu_payload["gyro_z"] = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 4) / 131.0
        temp_raw_mpu = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_TEMP_OUT_H)
        imu_payload["temperature_mpu"] = temp_raw_mpu / 340.0 + 36.53
    except Exception as e:
        print(f"Error reading MPU6050: {e}")

    try: # HMC5883L (Magnetometer data is typically X, Z, Y order from registers)
        # Read MSB then LSB for each axis
        mag_x_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB)
        mag_z_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 2)
        mag_y_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 4)
        # Apply scale factor (example, may need adjustment based on gain setting in REG_CONFIG_B)
        # Default gain gives ~0.92 mG/LSb = 92 nT/LSb
        # You'll need to calibrate this sensor properly.
        imu_payload["mag_x"] = mag_x_raw * 0.92 
        imu_payload["mag_y"] = mag_y_raw * 0.92 
        imu_payload["mag_z"] = mag_z_raw * 0.92
    except Exception as e:
        print(f"Error reading HMC5883L: {e}")

    try: # BMP180
        # Read uncompensated temperature
        bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP)
        time.sleep(0.005) # Wait for measurement
        UT = read_unsigned_word_big_endian(bus, BMP180_ADDR, BMP180_REG_RESULT_MSB)

        # Read uncompensated pressure (OSS=0 for this example)
        OSS = 0 # Oversampling setting (0,1,2,3)
        bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_PRESSURE_0 + (OSS << 6))
        time.sleep(0.005 + (0.003 * (1<<OSS))) # Wait based on OSS
        UP_msb = bus.read_byte_data(BMP180_ADDR, 0xF6)
        UP_lsb = bus.read_byte_data(BMP180_ADDR, 0xF7)
        UP_xlsb = bus.read_byte_data(BMP180_ADDR, 0xF8)
        UP = ((UP_msb << 16) + (UP_lsb << 8) + UP_xlsb) >> (8 - OSS)


        # Calculate true temperature
        X1 = (UT - AC6) * AC5 / (2**15)
        X2 = MC * (2**11) / (X1 + MD)
        B5 = X1 + X2
        true_temp_c = (B5 + 8) / (2**4) / 10.0 # Temperature in Celsius
        imu_payload["temperature_bmp"] = true_temp_c

        # Calculate true pressure
        B6 = B5 - 4000
        X1 = (B2 * (B6 * B6 / (2**12))) / (2**11)
        X2 = AC2 * B6 / (2**11)
        X3 = X1 + X2
        B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4
        X1 = AC3 * B6 / (2**13)
        X2 = (B1 * (B6 * B6 / (2**12))) / (2**16)
        X3 = ((X1 + X2) + 2) / (2**2)
        B4 = AC4 * (X3 + 32768) / (2**15)
        B7 = (UP - B3) * (50000 >> OSS)
        if B7 < 0x80000000:
            p = (B7 * 2) / B4
        else:
            p = (B7 / B4) * 2
        X1 = (p / (2**8)) * (p / (2**8))
        X1 = (X1 * 3038) / (2**16)
        X2 = (-7357 * p) / (2**16)
        pressure_pa = p + (X1 + X2 + 3791) / (2**4) # Pressure in Pascals
        imu_payload["pressure"] = pressure_pa
        
        # Calculate altitude (example, assumes sea level pressure = 101325 Pa)
        altitude = 44330 * (1.0 - pow(pressure_pa / 101325.0, 1/5.255))
        imu_payload["altitude_bmp"] = altitude

    except Exception as e:
        print(f"Error reading BMP180: {e}")

    return imu_payload

# ---------- PID Controller (Ví dụ đơn giản cho Yaw) ----------
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(-255, 255)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.output_limits = output_limits
        self.last_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            return 0 # Hoặc giá trị điều khiển trước đó

        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Clamp output
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        self.prev_error = error
        self.last_time = current_time
        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.prev_error = 0 # Reset error and integral when setpoint changes
        self.integral = 0
        self.last_time = time.time()


# ---------- Helper Functions ----------
def setup_serial_to_master():
    global ser_master
    try:
        print(f"Connecting to Arduino Master on {SERIAL_PORT_MASTER}...")
        ser_master = serial.Serial(SERIAL_PORT_MASTER, BAUD_RATE_MASTER, timeout=0.1)
        time.sleep(2) # Wait for connection
        print("Serial connection to Master established for Repeat phase.")
        return True
    except Exception as e:
        print(f"Failed to connect to Arduino Master: {e}")
        ser_master = None
        return False

def send_motor_command_to_master(left_dir, left_speed, right_dir, right_speed):
    if ser_master and ser_master.is_open:
        # Master Arduino của bạn đang chờ lệnh dạng "9:MOTOR:L_IDX:L_DIR:L_SPD"
        # Chúng ta cần gửi 2 lệnh riêng
        cmd_left = f"9:MOTOR:0:{left_dir}:{int(left_speed)}"
        cmd_right = f"9:MOTOR:1:{right_dir}:{int(right_speed)}"
        try:
            # print(f"Repeat sending: {cmd_left}") # Debug
            ser_master.write(cmd_left.encode() + b'\n')
            time.sleep(0.01) # Chờ một chút giữa các lệnh
            # print(f"Repeat sending: {cmd_right}") # Debug
            ser_master.write(cmd_right.encode() + b'\n')
        except Exception as e:
            print(f"Error sending motor command to master: {e}")

def send_servo_command_to_master(servo_idx, angle):
    if ser_master and ser_master.is_open:
        cmd = f"9:SERVO:{servo_idx}:{int(angle)}"
        try:
            # print(f"Repeat sending: {cmd}") # Debug
            ser_master.write(cmd.encode() + b'\n')
        except Exception as e:
            print(f"Error sending servo command to master: {e}")

def calculate_orientation(accel_data, gyro_data, mag_data, dt):
    # Đây là nơi bạn sẽ triển khai thuật toán sensor fusion (ví dụ: Madgwick, Mahony, Complementary)
    # Để đơn giản ban đầu, bạn có thể chỉ dùng yaw từ magnetometer (cần hiệu chỉnh cẩn thận)
    # hoặc tính toán roll/pitch từ accelerometer (chỉ chính xác khi không có gia tốc ngang)
    # và yaw rate từ gyroscope.
    
    # Ví dụ RẤT ĐƠN GIẢN (chỉ dùng cho minh họa, không chính xác khi có chuyển động):
    # Roll từ accelerometer
    # roll_acc = math.atan2(accel_data['accel_y'], accel_data['accel_z']) * 180 / math.pi
    # Pitch từ accelerometer
    # pitch_acc = math.atan2(-accel_data['accel_x'], math.sqrt(accel_data['accel_y']**2 + accel_data['accel_z']**2)) * 180 / math.pi

    # Yaw từ magnetometer (cần hiệu chỉnh tilt và nhiễu từ)
    # Ví dụ: heading = math.atan2(mag_data['mag_y'], mag_data['mag_x']) * 180 / math.pi
    # if heading < 0: heading += 360

    # Trong thực tế, bạn cần một bộ lọc tốt hơn.
    # Giả sử bạn có một hàm trả về yaw, pitch, roll đã được lọc:
    # current_yaw, current_pitch, current_roll = your_sensor_fusion_function(accel, gyro, mag, dt)
    
    # Tạm thời trả về giá trị giả định hoặc chỉ gyro_z cho yaw_rate
    # Để chạy được, bạn cần cung cấp một giá trị yaw thực tế ở đây
    # Ví dụ, chỉ lấy yaw_rate từ gyro z (độ/giây)
    # return {"yaw_rate": gyro_data.get("gyro_z", 0), "pitch": 0, "roll": 0, "yaw": 0}
    
    # --- Cần một hàm sensor fusion thực sự ở đây ---
    # Giả sử, chúng ta chỉ quan tâm yaw và dùng gyro_z để tích phân (rất dễ trôi)
    # Đây chỉ là ví dụ để code chạy được, bạn cần thay thế bằng giải thuật tốt hơn.
    global current_estimated_yaw
    yaw_increment = gyro_data.get("gyro_z", 0) * dt # gyro_z là deg/s
    current_estimated_yaw = (current_estimated_yaw + yaw_increment) % 360
    if current_estimated_yaw < 0: current_estimated_yaw += 360

    return {"yaw": current_estimated_yaw, "pitch": 0, "roll": 0} # Trả về yaw ước tính


current_estimated_yaw = 0 # Biến toàn cục để ước tính yaw (cần khởi tạo hoặc đọc từ IMU)

# ---------- MAIN REPEAT LOGIC ----------
def main_repeat(session_path):
    global current_estimated_yaw
    log_file_path = os.path.join(session_path, "data.csv")
    if not os.path.exists(log_file_path):
        print(f"Log file not found: {log_file_path}")
        return

    print(f"Starting Repeat phase for session: {session_path}")
    setup_imu_for_repeat() # Copy đầy đủ hàm get_imu_data và các hàm read_word từ teach.py vào đây
    
    # Khởi tạo lại current_estimated_yaw từ IMU nếu có thể, hoặc từ 0
    # Ví dụ, đọc magnetometer để có yaw ban đầu (cần hiệu chỉnh)
    # initial_mag_data = get_imu_data() # Giả sử get_imu_data đã được copy và hoạt động
    # current_estimated_yaw = calculate_initial_yaw_from_magnetometer(initial_mag_data)

    # PID controller cho Yaw (điều chỉnh các giá trị Kp, Ki, Kd)
    # Ví dụ: Kp=1.5, Ki=0.1, Kd=0.05. Giới hạn output là +/- 50 (thay đổi tốc độ motor)
    yaw_pid = PIDController(Kp=1.5, Ki=0.1, Kd=0.05, output_limits=(-80, 80))

    last_time_stamp_from_log = None
    start_repeat_time = time.time()

    with open(log_file_path, mode='r') as f:
        reader = csv.DictReader(f)
        for row_idx, row in enumerate(reader):
            try:
                # Lấy dữ liệu từ log
                log_timestamp = float(row["timestamp"])
                # log_frame_file = row["frame_file"] # Có thể dùng để hiển thị hoặc xử lý ảnh sau này
                
                # Lấy lệnh điều khiển cơ bản từ log
                target_motor_left_dir = int(float(row["motor_left_dir"]))
                target_motor_left_speed = int(float(row["motor_left_speed"]))
                target_motor_right_dir = int(float(row["motor_right_dir"]))
                target_motor_right_speed = int(float(row["motor_right_speed"]))
                target_servo1_angle = int(float(row["servo1_angle"]))
                target_servo2_angle = int(float(row["servo2_angle"]))

                # Lấy dữ liệu IMU mục tiêu từ log (ví dụ: yaw)
                # Bạn cần quyết định dữ liệu IMU nào từ log sẽ là setpoint.
                # Ví dụ, nếu bạn có một cột "target_yaw" trong log (tính từ fusion khi teach):
                # target_yaw_from_log = float(row["target_yaw"])
                # Hoặc nếu bạn muốn xe lặp lại chuyển động tương đối, bạn sẽ dùng gyro data
                # từ log để tái tạo yaw_rate.
                # Để đơn giản, giả sử chúng ta muốn giữ một yaw cố định (ví dụ 0 độ)
                # hoặc một yaw được tính toán từ dữ liệu IMU đã log.
                # Giả sử chúng ta log accel_x/y/z, gyro_x/y/z, mag_x/y/z
                # và chúng ta muốn tái tạo yaw.
                # Đây là phần phức tạp, vì "yaw" không được log trực tiếp.
                # Bạn cần một cột "log_yaw" trong CSV hoặc tính toán nó.
                # Tạm thời, chúng ta bỏ qua việc dùng yaw từ log làm setpoint trực tiếp
                # mà sẽ cố gắng chạy theo lệnh motor và dùng PID để giữ hướng.

                # --- Xử lý thời gian ---
                if last_time_stamp_from_log is not None:
                    delay_from_log = log_timestamp - last_time_stamp_from_log
                    # Bù trừ thời gian thực thi của vòng lặp Python
                    current_processing_time = time.time() - (start_repeat_time + (log_timestamp - float(reader.fieldnames[0]))) # Cần timestamp đầu tiên
                    actual_delay = delay_from_log - current_processing_time
                    if actual_delay > 0:
                        time.sleep(actual_delay)
                
                if row_idx == 0: # Lưu timestamp đầu tiên của log
                    first_log_timestamp = log_timestamp
                
                # Cập nhật thời gian bắt đầu tương đối cho lần lặp này
                current_loop_target_time_offset = log_timestamp - first_log_timestamp


                last_time_stamp_from_log = log_timestamp
                current_time_for_dt = time.time() # Dùng cho dt của PID và sensor fusion


                # --- Đọc IMU hiện tại ---
                live_imu_data_dict = get_imu_data() # Đảm bảo hàm này đã được copy từ teach.py và hoạt động
                
                # --- Tính toán Orientation hiện tại ---
                # dt_imu = current_time_for_dt - yaw_pid.last_time # dt cho sensor fusion
                # current_orientation = calculate_orientation(
                #     {"accel_x": live_imu_data_dict.get("accel_x"), ...}, # Truyền đúng cấu trúc
                #     {"gyro_z": live_imu_data_dict.get("gyro_z"), ...},
                #     {"mag_x": live_imu_data_dict.get("mag_x"), ...},
                #     dt_imu
                # )
                # current_live_yaw = current_orientation["yaw"]
                
                # Để chạy thử, ta giả định current_live_yaw được cập nhật trong calculate_orientation
                # và `current_estimated_yaw` là biến toàn cục được cập nhật.
                # Bạn cần một hàm `get_current_fused_orientation()` tốt hơn.
                dt_for_fusion = current_time_for_dt - getattr(calculate_orientation, 'last_call_time', current_time_for_dt)
                if dt_for_fusion <= 0: dt_for_fusion = 0.01 # Tránh dt = 0
                calculate_orientation.last_call_time = current_time_for_dt
                
                live_orientation = calculate_orientation( # Truyền dữ liệu vào
                     live_imu_data_dict, # Giả sử hàm này chấp nhận dict
                     live_imu_data_dict,
                     live_imu_data_dict,
                     dt_for_fusion
                )
                current_live_yaw = live_orientation["yaw"] # Dùng yaw ước tính từ gyro


                # --- Logic điều khiển với PID (ví dụ cho Yaw) ---
                # Setpoint cho PID có thể là yaw từ log (nếu có) hoặc một giá trị cố định.
                # Hoặc, nếu không có yaw mục tiêu từ log, PID có thể cố gắng giữ hướng
                # dựa trên thay đổi yaw_rate từ gyro đã log.
                # Ví dụ đơn giản: PID cố gắng giữ xe đi thẳng (yaw không đổi từ lúc bắt đầu)
                # Hoặc, nếu `target_motor_left_speed != target_motor_right_speed`, xe đang rẽ,
                # lúc đó setpoint yaw của PID nên được cập nhật theo gyro_z đã log.
                
                # Giả sử chúng ta muốn xe giữ hướng 0 độ khi đi thẳng,
                # và khi rẽ thì không dùng PID yaw hoặc thay đổi setpoint.
                # Điều này cần logic phức tạp hơn.

                # Đơn giản nhất: PID luôn cố gắng giữ yaw hiện tại = setpoint.
                # Setpoint ban đầu có thể là 0 hoặc yaw ban đầu của xe.
                # Hoặc setpoint là target_yaw_from_log (nếu bạn thêm cột đó vào CSV)
                # yaw_pid.set_setpoint(target_yaw_from_log) # Nếu có
                
                # Ví dụ: nếu xe dự kiến đi thẳng (tốc độ 2 bánh bằng nhau và cùng chiều)
                # thì PID sẽ cố gắng giữ yaw ổn định.
                yaw_correction = 0
                if target_motor_left_dir == target_motor_right_dir and target_motor_left_speed > 0 and target_motor_right_speed > 0:
                    # Nếu đang đi thẳng, PID cố gắng giữ yaw. Setpoint có thể là yaw ở đầu đoạn thẳng đó.
                    # Hoặc, nếu bạn không có yaw tuyệt đối từ log, bạn có thể dùng PID để giảm yaw_rate (gyro_z) về 0.
                    # yaw_pid.set_setpoint(0) # Giữ yaw_rate = 0
                    # yaw_correction = yaw_pid.update(live_imu_data_dict.get("gyro_z", 0)) # Update PID với gyro_z

                    # Nếu muốn giữ yaw tuyệt đối (cần target_yaw_from_log)
                    # yaw_pid.set_setpoint(TARGET_YAW_FROM_LOG_OR_CALCULATED)
                    # yaw_correction = yaw_pid.update(current_live_yaw)
                    pass # Phần này cần làm rõ cách bạn muốn dùng yaw từ log

                # Áp dụng yaw_correction vào tốc độ motor
                # Nếu yaw_correction > 0, nghĩa là cần rẽ phải để đạt setpoint
                # -> giảm bánh phải, tăng bánh trái (nếu đi tới)
                # -> tăng bánh phải, giảm bánh trái (nếu đi lùi)
                
                final_left_speed = target_motor_left_speed
                final_right_speed = target_motor_right_speed

                # Logic đơn giản hóa: (Cần xem lại cẩn thận chiều của correction)
                # if target_motor_left_dir == FORWARD:
                #     final_left_speed -= yaw_correction
                #     final_right_speed += yaw_correction
                # elif target_motor_left_dir == BACKWARD:
                #     final_left_speed += yaw_correction # Chú ý dấu
                #     final_right_speed -= yaw_correction

                # Giữ tốc độ trong khoảng 0-255
                final_left_speed = max(0, min(255, final_left_speed))
                final_right_speed = max(0, min(255, final_right_speed))

                # Gửi lệnh motor cuối cùng
                send_motor_command_to_master(
                    target_motor_left_dir, final_left_speed,
                    target_motor_right_dir, final_right_speed
                )

                # Gửi lệnh servo
                send_servo_command_to_master(0, target_servo1_angle)
                send_servo_command_to_master(1, target_servo2_angle)
                
                # Đồng bộ thời gian với log
                # Tính thời gian đã trôi qua trong thực tế so với log
                elapsed_real_time = time.time() - start_repeat_time
                elapsed_log_time = log_timestamp - first_log_timestamp
                
                sleep_duration = elapsed_log_time - elapsed_real_time
                if sleep_duration > 0:
                    time.sleep(sleep_duration)
                # Nếu sleep_duration < 0, nghĩa là bị trễ so với log.
                
                if (row_idx + 1) % 10 == 0: # In thông báo mỗi 10 dòng
                    print(f"Repeat: Processed log entry {row_idx+1}, "
                          f"Target L: {target_motor_left_dir}/{target_motor_left_speed}, "
                          f"R: {target_motor_right_dir}/{target_motor_right_speed}. "
                          f"LiveYawEst: {current_live_yaw:.2f}")


            except ValueError as e:
                print(f"Skipping row due to data error: {row} - {e}")
            except KeyboardInterrupt:
                print("\nRepeat phase interrupted by user.")
                break
            except Exception as e:
                print(f"Error during repeat loop for row {row}: {e}")
                # Có thể thêm break ở đây nếu lỗi nghiêm trọng
    
    print("Repeat phase finished.")
    # Dừng motor sau khi hoàn thành
    send_motor_command_to_master(STOP, 0, STOP, 0)


if __name__ == "__main__":
    # ----- Để chạy TEACH -----
    # main_teach()
    # print("Teach phase finished. Run repeat phase next if desired.")

    # ----- Để chạy REPEAT -----
    # Bạn cần cung cấp đường dẫn đến thư mục session đã ghi
    # Ví dụ: dataset_teach/20250510_203000
    # session_to_repeat = "dataset_teach/YYYYMMDD_HHMMSS" # THAY THẾ BẰNG SESSION CỤ THỂ
    
    # Tìm session mới nhất để chạy repeat (ví dụ)
    dataset_dir = "dataset_teach"
    if not os.path.isdir(dataset_dir):
        print(f"Dataset directory '{dataset_dir}' not found. Run teach phase first.")
        sys.exit(1)

    all_sessions = [d for d in os.listdir(dataset_dir) if os.path.isdir(os.path.join(dataset_dir, d))]
    if not all_sessions:
        print(f"No sessions found in '{dataset_dir}'. Run teach phase first.")
        sys.exit(1)
    
    latest_session = sorted(all_sessions)[-1] # Sắp xếp theo tên (timestamp)
    session_to_repeat = os.path.join(dataset_dir, latest_session)
    print(f"Attempting to run Repeat phase for latest session: {session_to_repeat}")

    # Khởi tạo hàm get_imu_data() và các hàm phụ trợ từ teach.py vào repeat.py
    # Đảm bảo setup_imu_for_repeat() được gọi trước get_imu_data() trong repeat.py
    # Chép các hàm: setup_imu, read_signed_word, read_unsigned_word,
    # read_signed_word_big_endian, read_unsigned_word_big_endian, get_imu_data
    # từ teach.py vào script repeat.py này.
    
    # Đây là phần bạn cần chép các hàm IMU từ teach.py vào đây:
    # def setup_imu(): ... (đã đổi tên thành setup_imu_for_repeat)
    # def read_signed_word(bus_obj, addr, reg): ...
    # def read_unsigned_word(bus_obj, addr, reg): ...
    # def read_signed_word_big_endian(bus_obj, addr, reg_msb): ...
    # def read_unsigned_word_big_endian(bus_obj, addr, reg_msb): ...
    # def get_imu_data(): ... (hàm này phải được định nghĩa ở đây)
    # global AC1, AC2, AC3, B1, B2, MB, MC, MD, VB1, VB2, AC4, AC5, AC6 # Khai báo lại nếu cần
    
    # ****** SAO CHÉP CÁC HÀM IMU TỪ TEACH.PY VÀO ĐÂY ******
    # Tạm thời để chạy, tôi sẽ định nghĩa một hàm get_imu_data giả
    def get_imu_data_placeholder(): # BẠN PHẢI THAY BẰNG HÀM THỰC TẾ TỪ TEACH.PY
        return {
            "accel_x": 0, "accel_y": 0, "accel_z": 9.8,
            "gyro_x": 0, "gyro_y": 0, "gyro_z": 0.1, # Giả sử có chút yaw rate
            "mag_x": 0.2, "mag_y": 0, "mag_z": 0,
            "temperature_mpu": 25, "pressure": 101325, "altitude_bmp": 0, "temperature_bmp": 25
        }
    # Gán lại hàm get_imu_data để dùng placeholder (NHỚ THAY BẰNG HÀM THẬT)
    global get_imu_data 
    get_imu_data = get_imu_data_placeholder # XÓA DÒNG NÀY KHI BẠN ĐÃ COPY HÀM THẬT

    if setup_serial_to_master():
        main_repeat(session_to_repeat)
    else:
        print("Could not connect to Arduino Master for Repeat phase.")

    if ser_master and ser_master.is_open:
        ser_master.close()
        print("Serial connection to Master closed.")