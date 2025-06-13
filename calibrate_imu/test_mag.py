import math
import time
import smbus # For I2C with IMU
from math import atan2, sqrt, degrees, sin, cos


import json
import numpy as np

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






def calculate_tilt_compensated_heading(accel_x, accel_y, accel_z, mag_x, mag_y, mag_z):
    # Step 1: Tính pitch và roll
    pitch = atan2(-accel_x, sqrt(accel_y**2 + accel_z**2))
    roll = atan2(accel_y, accel_z)

    # Step 2: Bù nghiêng từ trường
    mag_x_comp = mag_x * cos(pitch) + mag_z * sin(pitch)
    mag_y_comp = (
        mag_x * sin(roll) * sin(pitch)
        + mag_y * cos(roll)
        - mag_z * sin(roll) * cos(pitch)
    )

    # Step 3: Tính hướng
    heading = atan2(-mag_y_comp, mag_x_comp)

    # Step 4: Chuyển sang độ
    heading_deg = degrees(heading)
    if heading_deg < 0:
        heading_deg += 360

    return heading_deg





def load_calibration_params(filename="imu_calibration_params.json"):
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except:
        print("Warning: IMU calibration file not found or unreadable.")
        return {
            "gyro_bias_x": 0.0, "gyro_bias_y": 0.0, "gyro_bias_z": 0.0,
            "hard_iron_offset_x": 0.0, "hard_iron_offset_y": 0.0, "hard_iron_offset_z": 0.0,
            "soft_iron_matrix": np.identity(3).tolist()
        }

calib = load_calibration_params()







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
        imu_payload["gyro_x"] = (read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H) / 131.0) - calib["gyro_bias_x"]
        imu_payload["gyro_y"] = (read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 2) / 131.0) - calib["gyro_bias_y"]
        imu_payload["gyro_z"] = (read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 4) / 131.0) - calib["gyro_bias_z"]
        temp_raw_mpu = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_TEMP_OUT_H)
        imu_payload["temperature_mpu"] = temp_raw_mpu / 340.0 + 36.53
    except Exception as e: pass # print(f"Teach: Error reading MPU6050: {e}")

    try: # HMC5883L
        mag_x_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB)
        mag_z_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 2) # Z is usually next
        mag_y_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 4) # Then Y

        # 1. Trừ hard-iron offset
        mag = np.array([
            mag_x_raw - calib["hard_iron_offset_x"],
            mag_y_raw - calib["hard_iron_offset_y"],
            mag_z_raw - calib["hard_iron_offset_z"]
        ])

        # 2. Nhân với soft-iron matrix (nếu bạn đã tính chính xác, còn không thì vẫn dùng identity matrix như mặc định)
        soft_matrix = np.array(calib["soft_iron_matrix"])
        mag_corrected = np.dot(soft_matrix, mag)

        imu_payload["mag_x"], imu_payload["mag_y"], imu_payload["mag_z"] = mag_corrected[0] * 0.92, mag_corrected[1] * 0.92, mag_corrected[2] * 0.92

    except Exception as e: pass # print(f"Teach: Error reading HMC5883L: {e}")

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
            imu_payload["temperature_bmp"], imu_payload["pressure"], imu_payload["altitude_bmp"] = 0, 0, 0
        else:
            X1 = X1_temp
            # Sử dụng // cho phép chia nguyên
            X2 = MC * (2**11) // (X1 + MD)
            B5 = X1 + X2
            # Nhiệt độ có thể giữ dạng float
            imu_payload["temperature_bmp"] = (B5 + 8) / (2**4) / 10.0
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
            B7 = (UP - B3) * (50000 >> OSS)
            if B4 == 0: 
                imu_payload["pressure"], imu_payload["altitude_bmp"] = 0, 0
            else:
                if B7 < 0x80000000:
                    pressure = (B7 * 2) // B4
                else:
                    pressure = (B7 // B4) * 2

                X1 = (pressure // 256) ** 2
                X1 = (X1 * 3038) // (2**16)
                X2 = (-7357 * pressure) // (2**16)
                pressure = pressure + ((X1 + X2 + 3791) // 16)
                imu_payload["pressure"] = pressure
                imu_payload["altitude_bmp"] = 44330.0 * (1.0 - (pressure / 101325.0) ** (1.0 / 5.255))


    except Exception as e:
        print(f"Lỗi khi đọc BMP180: {e}")
        # Có thể đặt giá trị mặc định ở đây nếu muốn
        imu_payload["temperature_bmp"], imu_payload["pressure"], imu_payload["altitude_bmp"] = 0, 0, 0
    imu_payload["heading"] = calculate_tilt_compensated_heading(imu_payload["accel_x"], imu_payload["accel_y"], imu_payload["accel_z"], imu_payload["mag_x"], imu_payload["mag_y"], imu_payload["mag_z"])

    return imu_payload


setup_imu()




calib = load_calibration_params()












a = get_imu_data()

initial_heading = a['heading']





while(1):
    x = get_imu_data() 
    # print(f"X = {x['mag_x']:.0f}, Y = {x['mag_y']:.0f}, Z = {x['mag_z']:.0f}")
    he = x['heading']
    he -= initial_heading
    if he < 0:
        he += 360
    print(f"{he:.0f}")