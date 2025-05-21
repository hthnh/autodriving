# teach.py
# Logs IMU data, Camera images, Control Commands (from Redis), and IPS data (from Redis).

import os
import time
import csv
from picamera2 import Picamera2
import smbus # For I2C with IMU
import redis # For IPC with control_processor.py and ips.py
import select # For non-blocking socket/stream read (not directly used with redis-py xread)
import sys
# import cv2 # Uncomment if you use cv2.flip and have it installed

# ---------- Redis Configuration ----------
REDIS_HOST = 'localhost'
REDIS_PORT = 6379
redis_client = None

# Streams to consume from
CONTROL_STREAM_NAME = 'vehicle:commands_processed'
IPS_STREAM_NAME = 'ips:data' # Assuming ips.py publishes to this stream

# Consumer group names (can be unique to teach.py or shared if appropriate)
TEACH_CONSUMER_GROUP_CONTROL = 'teach_group_control'
TEACH_CONSUMER_GROUP_IPS = 'teach_group_ips'
TEACH_CONSUMER_NAME = 'teach_instance_1'

# Variables to store the latest received data from Redis
latest_control_commands = {
    "motor_left_dir": 0, "motor_left_speed": 0,
    "motor_right_dir": 0, "motor_right_speed": 0,
    "servo1_angle": 90, "servo2_angle": 90,
    "e_stop_active": 0
}
latest_ips_data = {
    "ips_x": None, "ips_y": None, "ips_yaw": None, # Use None for null/not available
    "ips_quality": None
}
# Last read IDs for Redis streams to avoid reprocessing old messages if script restarts without group
# last_control_id = '0-0' # Start from beginning, or use '$' for only new after start
# last_ips_id = '0-0'

# ---------- IMU GY-87 SETUP (Copy from your working teach.py or nrf_receiver.py) ----------
# Ensure all necessary I2C addresses and registers are defined here
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

AC1, AC2, AC3, B1, B2, MB, MC, MD = 0,0,0,0,0,0,0,0
AC4, AC5, AC6 = 0,0,0

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

def setup_imu():
    global AC1, AC2, AC3, B1, B2, MB, MC, MD, AC4, AC5, AC6 # Ensure these are global for modification
    print("Teach: Initializing IMU...")
    try: bus.write_byte_data(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00)
    except Exception as e: print(f"Teach: Failed to wake MPU6050: {e}")
    try:
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70)
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 0xA0)
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00)
    except Exception as e: print(f"Teach: Failed to configure HMC5883L: {e}")
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
        print("Teach: IMU BMP180 calibration data read.")
    except Exception as e: print(f"Teach: Failed to read BMP180 calibration data: {e}")
    print("Teach: IMU setup complete.")

def get_imu_data():
    # This function should be identical to the one you finalized for teach.py
    # that reads MPU6050, HMC5883L, and BMP180.
    # For brevity, I'll use a placeholder structure.
    # MAKE SURE TO COPY YOUR FULL WORKING get_imu_data() HERE.
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
    except Exception as e: pass # print(f"Teach: Error reading MPU6050: {e}")

    try: # HMC5883L
        mag_x_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB)
        mag_z_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 2) # Z is usually next
        mag_y_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 4) # Then Y
        imu_payload["mag_x"], imu_payload["mag_y"], imu_payload["mag_z"] = mag_x_raw * 0.92, mag_y_raw * 0.92, mag_z_raw * 0.92
    except Exception as e: pass # print(f"Teach: Error reading HMC5883L: {e}")

    try: # BMP180
        bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP)
        time.sleep(0.005)
        UT = read_unsigned_word_big_endian(bus, BMP180_ADDR, BMP180_REG_RESULT_MSB)
        OSS = 0
        bus.write_byte_data(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_PRESSURE_0 + (OSS << 6))
        time.sleep(0.005 + (0.003 * (1<<OSS)))
        UP_msb = bus.read_byte_data(BMP180_ADDR, 0xF6)
        UP_lsb = bus.read_byte_data(BMP180_ADDR, 0xF7)
        UP_xlsb = bus.read_byte_data(BMP180_ADDR, 0xF8)
        UP = ((UP_msb << 16) + (UP_lsb << 8) + UP_xlsb) >> (8 - OSS)
        X1 = (UT - AC6) * AC5 / (2**15); X2 = MC * (2**11) / (X1 + MD if (X1 + MD) != 0 else 1) ; B5 = X1 + X2
        imu_payload["temperature_bmp"] = (B5 + 8) / (2**4) / 10.0
        B6 = B5 - 4000; X1 = (B2 * (B6 * B6 / (2**12))) / (2**11); X2 = AC2 * B6 / (2**11); X3 = X1 + X2
        B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4
        X1 = AC3 * B6 / (2**13); X2 = (B1 * (B6 * B6 / (2**12))) / (2**16); X3 = ((X1 + X2) + 2) / (2**2)
        B4 = AC4 * (X3 + 32768) / (2**15); B7 = (UP - B3) * (50000 >> OSS)
        p = (B7 * 2) / B4 if B4 != 0 and B7 >= 0x80000000 else (B7 / B4) * 2 if B4 != 0 else 0
        X1 = (p / (2**8))**2; X1 = (X1 * 3038) / (2**16); X2 = (-7357 * p) / (2**16)
        imu_payload["pressure"] = p + (X1 + X2 + 3791) / (2**4)
        imu_payload["altitude_bmp"] = 44330 * (1.0 - pow(imu_payload["pressure"] / 101325.0, 1/5.255)) if imu_payload["pressure"] > 0 else 0
    except Exception as e: pass # print(f"Teach: Error reading BMP180: {e}")
    return imu_payload

# ---------- RECORDING CLASS ----------
class DataRecorder:
    def __init__(self, save_path_base, use_ips_flag):
        self.session_name = time.strftime("%Y%m%d_%H%M%S")
        self.save_path = os.path.join(save_path_base, self.session_name)
        self.frame_dir = os.path.join(self.save_path, "frames")
        os.makedirs(self.frame_dir, exist_ok=True)
        self.use_ips = use_ips_flag

        self.csv_path = os.path.join(self.save_path, "data.csv")
        csv_headers = [
            "timestamp", "frame_file",
            "accel_x", "accel_y", "accel_z",
            "gyro_x", "gyro_y", "gyro_z",
            "mag_x", "mag_y", "mag_z",
            "temp_mpu", "temp_bmp", "pressure", "altitude_bmp",
            "motor_left_dir", "motor_left_speed",
            "motor_right_dir", "motor_right_speed",
            "servo1_angle", "servo2_angle", "e_stop_active"
        ]
        if self.use_ips:
            csv_headers.extend(["ips_x", "ips_y", "ips_yaw", "ips_quality"])
        
        with open(self.csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(csv_headers)
        
        self.picam2 = Picamera2(camera_num = 1)
        # Configure for faster still capture, adjust resolution as needed
        config = self.picam2.create_still_configuration(main={"size": (640, 480)}, lores={"size": (320, 240)}, display="lores")
        self.picam2.configure(config)
        self.picam2.start()
        print(f"Teach: Picamera2 started. Recording session: {self.session_name}")
        print(f"Teach: Saving data to: {self.save_path}")

    def record_frame(self, imu_data_dict, control_cmd_dict, ips_data_dict):
        timestamp = time.time()
        frame_filename = f"{timestamp:.3f}.jpg" # More precision for filename
        frame_path = os.path.join(self.frame_dir, frame_filename)
        
        try:
            # Capture to a buffer in a specific format if needed, or directly to file
            # For speed, capture_array might be faster then saving with PIL
            frame_array = self.picam2.capture_array("main") # Capture from main stream
            # if 'cv2' in sys.modules:
                # frame_array = cv2.flip(frame_array, -1) # Optional flip

            from PIL import Image
            img = Image.fromarray(frame_array)
            img.save(frame_path)
        except Exception as e:
            print(f"Teach: Error capturing/saving frame: {e}")
            frame_filename = "error.jpg"

        row_data = [
            timestamp, frame_filename,
            imu_data_dict.get("accel_x", 0), imu_data_dict.get("accel_y", 0), imu_data_dict.get("accel_z", 0),
            imu_data_dict.get("gyro_x", 0), imu_data_dict.get("gyro_y", 0), imu_data_dict.get("gyro_z", 0),
            imu_data_dict.get("mag_x", 0), imu_data_dict.get("mag_y", 0), imu_data_dict.get("mag_z", 0),
            imu_data_dict.get("temperature_mpu", 0), imu_data_dict.get("temperature_bmp", 0),
            imu_data_dict.get("pressure", 0), imu_data_dict.get("altitude_bmp", 0),
            control_cmd_dict.get("motor_left_dir", 0), control_cmd_dict.get("motor_left_speed", 0),
            control_cmd_dict.get("motor_right_dir", 0), control_cmd_dict.get("motor_right_speed", 0),
            control_cmd_dict.get("servo1_angle", 90), control_cmd_dict.get("servo2_angle", 90),
            control_cmd_dict.get("e_stop_active", 0)
        ]
        if self.use_ips:
            row_data.extend([
                ips_data_dict.get("ips_x"), # Will be None if not available
                ips_data_dict.get("ips_y"),
                ips_data_dict.get("ips_yaw"),
                ips_data_dict.get("ips_quality")
            ])

        with open(self.csv_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row_data)

    def close(self):
        if hasattr(self, 'picam2') and self.picam2:
            self.picam2.stop()
            print("Teach: Picamera2 stopped.")

# ---------- REDIS CLIENT SETUP and MAIN ----------



def read_redis_streams():
    global latest_control_commands, latest_ips_data, USE_IPS_FLAG # Đảm bảo USE_IPS_FLAG có thể truy cập

    if not redis_client:
        return

    # Tạo dictionary streams để đọc, luôn dùng '>' làm ID cho XREADGROUP
    streams_to_read_with_group = {}
    
    # Luôn đọc từ control stream nếu redis_client tồn tại
    streams_to_read_with_group[CONTROL_STREAM_NAME] = '>'
    
    if USE_IPS_FLAG:
        streams_to_read_with_group[IPS_STREAM_NAME] = '>'
    
    # Xử lý groupname cho xreadgroup khi đọc nhiều stream.
    # XREADGROUP chỉ cho phép đọc từ một group tại một thời điểm cho nhiều stream.
    # Điều này có nghĩa là tất cả các stream bạn đọc trong một lệnh XREADGROUP
    # phải thuộc về cùng một consumer group HOẶC bạn phải gọi XREADGROUP riêng cho từng group.
    # Nếu CONTROL_STREAM_NAME và IPS_STREAM_NAME dùng chung group thì ổn.
    # Nếu chúng dùng group khác nhau (TEACH_CONSUMER_GROUP_CONTROL và TEACH_CONSUMER_GROUP_IPS)
    # bạn cần gọi XREADGROUP hai lần.
    
    # Giả định hiện tại là mỗi stream có group riêng như đã setup:
    # TEACH_CONSUMER_GROUP_CONTROL cho CONTROL_STREAM_NAME
    # TEACH_CONSUMER_GROUP_IPS cho IPS_STREAM_NAME
    # Chúng ta sẽ đọc từng stream một nếu chúng thuộc các group khác nhau.

    # Đọc từ CONTROL_STREAM_NAME
    try:
        control_messages = redis_client.xreadgroup(
            groupname=TEACH_CONSUMER_GROUP_CONTROL,
            consumername=TEACH_CONSUMER_NAME,
            streams={CONTROL_STREAM_NAME: '>'}, # Chỉ đọc từ stream này với ID '>'
            count=5, # Đọc tối đa 5 message mỗi lần
            block=1 # Block tối đa 1ms nếu không có message (để không làm treo vòng lặp chính quá lâu)
        )

        if control_messages:
            for stream_name_bytes, message_list in control_messages: # stream_name_bytes sẽ là CONTROL_STREAM_NAME
                for message_id_bytes, data_bytes_dict in message_list:
                    # print(f"Teach: Received from CONTROL (ID: {message_id_bytes.decode()}): {data_bytes_dict}") # Debug
                    latest_control_commands["motor_left_dir"] = int(data_bytes_dict.get(b'motor_left_dir', b'0'))
                    latest_control_commands["motor_left_speed"] = int(data_bytes_dict.get(b'motor_left_speed', b'0'))
                    latest_control_commands["motor_right_dir"] = int(data_bytes_dict.get(b'motor_right_dir', b'0'))
                    latest_control_commands["motor_right_speed"] = int(data_bytes_dict.get(b'motor_right_speed', b'0'))
                    latest_control_commands["servo1_angle"] = int(data_bytes_dict.get(b'servo1_angle', b'90'))
                    latest_control_commands["servo2_angle"] = int(data_bytes_dict.get(b'servo2_angle', b'90'))
                    latest_control_commands["e_stop_active"] = int(data_bytes_dict.get(b'e_stop_active', b'0'))
                    redis_client.xack(CONTROL_STREAM_NAME, TEACH_CONSUMER_GROUP_CONTROL, message_id_bytes)
    except redis.exceptions.RedisError as e:
        print(f"Teach: Redis error reading CONTROL stream: {e}")
    except Exception as e:
        print(f"Teach: Error processing CONTROL Redis messages: {e}")


    # Đọc từ IPS_STREAM_NAME nếu USE_IPS_FLAG là True
    if USE_IPS_FLAG:
        try:
            ips_messages = redis_client.xreadgroup(
                groupname=TEACH_CONSUMER_GROUP_IPS,
                consumername=TEACH_CONSUMER_NAME,
                streams={IPS_STREAM_NAME: '>'}, # Chỉ đọc từ stream này với ID '>'
                count=5,
                block=1 # Block tối đa 1ms
            )

            if ips_messages:
                for stream_name_bytes, message_list in ips_messages: # stream_name_bytes sẽ là IPS_STREAM_NAME
                    for message_id_bytes, data_bytes_dict in message_list:
                        # print(f"Teach: Received from IPS (ID: {message_id_bytes.decode()}): {data_bytes_dict}") # Debug
                        latest_ips_data["ips_x"] = float(data_bytes_dict.get(b'ips_x', b'0.0'))
                        latest_ips_data["ips_y"] = float(data_bytes_dict.get(b'ips_y', b'0.0'))
                        latest_ips_data["ips_yaw"] = float(data_bytes_dict.get(b'ips_yaw', b'0.0'))
                        latest_ips_data["ips_quality"] = float(data_bytes_dict.get(b'ips_quality', b'0.0'))
                        redis_client.xack(IPS_STREAM_NAME, TEACH_CONSUMER_GROUP_IPS, message_id_bytes)
        except redis.exceptions.RedisError as e:
            print(f"Teach: Redis error reading IPS stream: {e}")
        except Exception as e:
            print(f"Teach: Error processing IPS Redis messages: {e}")

# Trong hàm setup_redis_for_teach():
# Bỏ các dòng gán last_control_id = '$' và last_ips_id = '$' đi.
# Việc tạo group với id='0' là đúng để group bắt đầu theo dõi từ đầu stream (nếu stream mới hoặc consumer mới).
# Sau đó, khi đọc, consumer sẽ dùng '>' để lấy message mới cho nó.

def setup_redis_for_teach():
    global redis_client # Bỏ last_control_id, last_ips_id khỏi global nếu không dùng nữa
    try:
        print("Teach: Connecting to Redis...")
        redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=False)
        redis_client.ping()
        print("Teach: Redis connection established.")

        # Setup for CONTROL stream
        try:
            redis_client.xgroup_create(name=CONTROL_STREAM_NAME, groupname=TEACH_CONSUMER_GROUP_CONTROL, id='0', mkstream=True)
            print(f"Teach: Consumer group '{TEACH_CONSUMER_GROUP_CONTROL}' ensured for '{CONTROL_STREAM_NAME}'.")
        except redis.exceptions.ResponseError as e:
            if "BUSYGROUP" in str(e): 
                print(f"Teach: Consumer group '{TEACH_CONSUMER_GROUP_CONTROL}' already exists for '{CONTROL_STREAM_NAME}'.")
            else: raise
        
        # Setup for IPS stream
        try:
            redis_client.xgroup_create(name=IPS_STREAM_NAME, groupname=TEACH_CONSUMER_GROUP_IPS, id='0', mkstream=True)
            print(f"Teach: Consumer group '{TEACH_CONSUMER_GROUP_IPS}' ensured for '{IPS_STREAM_NAME}'.")
        except redis.exceptions.ResponseError as e:
            if "BUSYGROUP" in str(e): 
                print(f"Teach: Consumer group '{TEACH_CONSUMER_GROUP_IPS}' already exists for '{IPS_STREAM_NAME}'.")
            else: raise
        return True
    except Exception as e:
        print(f"Teach: Failed to connect/setup Redis: {e}")
        return False


# --- Main Teach Phase Logic ---
SAVE_DIR_BASE = "T_R_navigation_method/dataset_teach"
# THIS FLAG CONTROLS IPS USAGE
USE_IPS_FLAG = False # Set to True if you have an ips.py publishing to IPS_STREAM_NAME

def main_teach_phase():
    global last_control_id, last_ips_id # Ensure these are accessible

    if not setup_redis_for_teach():
        print("Teach: Could not connect to Redis. Exiting teach phase.")
        return

    setup_imu()
    recorder = DataRecorder(save_path_base=SAVE_DIR_BASE, use_ips_flag=USE_IPS_FLAG)
    
    # Initialize last IDs to '>' to only get new messages after starting this consumer
    # Or, if you want to process messages this consumer might have missed if it restarted,
    # you'd read with '0' in xreadgroup and rely on XACK.
    # For simplicity, if this is the only consumer in the group, '>' for new messages is fine.
    # The XGROUP CREATE with '0' sets the starting point for the group.
    # When a consumer reads with '>', it gets messages not yet delivered to *any* consumer in that group.
    # If it's the only consumer, this means all new messages.
    # Let's adjust to use the group's perspective with '>'
    
    # For XREADGROUP, the ID in streams dict for XREADGROUP should be '>'
    # to get messages not yet delivered to this consumer in this group.
    # The `last_control_id` and `last_ips_id` are not strictly needed
    # if using XREADGROUP with '>' and XACKing.
    # The '0' in XGROUP CREATE establishes the group's starting point.
    # The '>' in XREADGROUP means "new messages for this consumer".

    print("[INFO] Teach phase started. Press Ctrl+C to stop.")
    log_interval = 0.1 # Log data every 0.1 seconds (10 Hz)
    last_log_time = time.time()

    try:
        while True:
            current_time = time.time()
            
            # Read from Redis streams non-blockingly or with short block
            read_redis_streams() # This updates latest_control_commands and latest_ips_data

            if current_time - last_log_time >= log_interval:
                imu_data = get_imu_data()



                print(f"--- IMU Data at {current_time:.2f} ---")
                print(f"  Accel (X,Y,Z): ({imu_data.get('accel_x', 0):.2f}, {imu_data.get('accel_y', 0):.2f}, {imu_data.get('accel_z', 0):.2f}) g")
                print(f"  Gyro (X,Y,Z):  ({imu_data.get('gyro_x', 0):.2f}, {imu_data.get('gyro_y', 0):.2f}, {imu_data.get('gyro_z', 0):.2f}) deg/s")
                print(f"  Mag (X,Y,Z):   ({imu_data.get('mag_x', 0):.2f}, {imu_data.get('mag_y', 0):.2f}, {imu_data.get('mag_z', 0):.2f}) uT") # Đơn vị có thể khác tùy scale factor
                print(f"  Temp MPU:      {imu_data.get('temperature_mpu', 0):.2f} C")
                print(f"  Temp BMP:      {imu_data.get('temperature_bmp', 0):.2f} C")
                print(f"  Pressure:      {imu_data.get('pressure', 0):.0f} Pa")
                print(f"  Altitude BMP:  {imu_data.get('altitude_bmp', 0):.2f} m")
                print(f"  Control CMDs:  {latest_control_commands}")
                if USE_IPS_FLAG:
                   print(f"  IPS Data:      {latest_ips_data}")
                print("-------------------------")




                recorder.record_frame(imu_data, latest_control_commands, latest_ips_data)
                # print(f"Logged at {current_time:.2f}") # Debug
                last_log_time = current_time
            
            # Adjust sleep to control overall loop rate, considering log_interval
            # If log_interval is 0.1, sleeping for 0.01 ensures Redis is checked frequently
            time.sleep(0.01) 

    except KeyboardInterrupt:
        print("\n[INFO] Teach phase stopped by user.")
    except Exception as e:
        print(f"[ERROR] An unexpected error occurred in main_teach_phase: {e}")
    finally:
        print("[INFO] Teach: Cleaning up...")
        if 'recorder' in locals() and recorder:
            recorder.close()
        if redis_client:
            # Optional: Remove consumer from group if desired, or just close connection
            # try:
            #     if redis_client.exists(CONTROL_STREAM_NAME):
            #        redis_client.xgroup_delconsumer(CONTROL_STREAM_NAME, TEACH_CONSUMER_GROUP_CONTROL, TEACH_CONSUMER_NAME)
            #     if USE_IPS_FLAG and redis_client.exists(IPS_STREAM_NAME):
            #        redis_client.xgroup_delconsumer(IPS_STREAM_NAME, TEACH_CONSUMER_GROUP_IPS, TEACH_CONSUMER_NAME)
            # except Exception as e_del_consumer:
            #     print(f"Teach: Error deleting consumer from group: {e_del_consumer}")
            redis_client.close() # Close the connection
            print("Teach: Redis connection closed.")
        print("[INFO] Teach phase cleanup complete.")

if __name__ == "__main__":
    # Example: Check for a command-line argument to enable IPS
    if len(sys.argv) > 1 and sys.argv[1].lower() == '--use-ips':
        USE_IPS_FLAG = True
        print("Teach: IPS data collection ENABLED via command-line argument.")
    else:
        USE_IPS_FLAG = False # Default
        print("Teach: IPS data collection DISABLED. To enable, run with --use-ips argument.")
        
    main_teach_phase()