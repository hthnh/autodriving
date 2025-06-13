# calibrate_imu.py
import smbus
import time
import math
import numpy as np
import json # Để lưu tham số hiệu chỉnh
import csv
import os

# --- Cấu hình I2C và địa chỉ cảm biến (Giống teach.py/repeat.py) ---
bus = smbus.SMBus(1)
MPU6050_ADDR = 0x68
MPU6050_REG_PWR_MGMT_1 = 0x6B
MPU6050_REG_GYRO_XOUT_H = 0x43

HMC5883L_ADDR = 0x1E
HMC5883L_REG_CONFIG_A = 0x00
HMC5883L_REG_CONFIG_B = 0x01 # Để điều chỉnh gain nếu cần
HMC5883L_REG_MODE = 0x02
HMC5883L_REG_DATA_X_MSB = 0x03

CALIBRATION_FILE = "imu_calibration_params.json"

# --- Các hàm đọc I2C (Sao chép từ teach.py/repeat.py) ---
def read_signed_word(bus_obj, addr, reg):
    high = bus_obj.read_byte_data(addr, reg)
    low = bus_obj.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    return value - 65536 if value >= 0x8000 else value

def read_signed_word_big_endian(bus_obj, addr, reg_msb):
    msb = bus_obj.read_byte_data(addr, reg_msb)
    lsb = bus_obj.read_byte_data(addr, reg_msb + 1)
    val = (msb << 8) + lsb
    return val - 65536 if val >= 0x8000 else val

# --- Chức năng 1: Hiệu chỉnh Gyroscope Bias ---
def calibrate_gyro(num_samples=5000):
    print("-" * 30)
    print("Gyroscope Bias Calibration")
    print("Please keep the robot perfectly STILL for a few seconds.")
    input("Press Enter to start...")

    gyro_data_sum = np.array([0.0, 0.0, 0.0])
    for _ in range(num_samples):
        try:
            gx = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H) / 131.0
            gy = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 2) / 131.0
            gz = read_signed_word(bus, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H + 4) / 131.0
            gyro_data_sum += np.array([gx, gy, gz])
        except Exception as e:
            print(f"Error reading gyro data during calibration: {e}")
            return None
        time.sleep(0.01) # Chờ giữa các lần đọc

    gyro_bias = gyro_data_sum / num_samples
    print(f"Gyroscope Bias (deg/s): X={gyro_bias[0]:.4f}, Y={gyro_bias[1]:.4f}, Z={gyro_bias[2]:.4f}")
    print("Gyroscope calibration complete.")
    return {"gyro_bias_x": gyro_bias[0], "gyro_bias_y": gyro_bias[1], "gyro_bias_z": gyro_bias[2]}

# --- Chức năng 2: Thu thập dữ liệu Magnetometer ---
def collect_magnetometer_data(duration_seconds=60, output_file="mag_raw_data.csv"):
    print("-" * 30)
    print("Magnetometer Data Collection")
    print(f"Please SLOWLY rotate the IMU/robot in all possible orientations (figure-8 motion in 3D space) for {duration_seconds} seconds.")
    print("Try to cover as many orientations as possible to get a good data cloud.")
    input("Press Enter to start data collection...")
    for i in range(10):
        print("wait:",i)
        time.sleep(1)

    start_time = time.time()
    mag_data_points = []
    
    # Cấu hình HMC5883L (có thể cần điều chỉnh gain ở HMC5883L_REG_CONFIG_B)
    try:
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70) # 8-sample avg, 15Hz (default)
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 0xA0) # Default gain +/- 4.7 Gauss
        bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00) # Continuous measurement
    except Exception as e:
        print(f"Error configuring HMC5883L: {e}")
        return None

    print("Collecting magnetometer data... Press Ctrl+C to stop early.")
    try:
        with open(output_file, 'w', newline='') as csvfile:
            fieldnames = ['mag_x_raw', 'mag_y_raw', 'mag_z_raw']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            while (time.time() - start_time) < duration_seconds:
                try:
                    # Đọc theo thứ tự X, Z, Y từ thanh ghi
                    mx_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB)
                    mz_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 2)
                    my_raw = read_signed_word_big_endian(bus, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB + 4)
                    
                    # Lưu ý: Trục của HMC5883L có thể khác với trục của MPU6050.
                    # Bạn cần xác định đúng trục X, Y, Z của magnetometer so với robot.
                    # Ở đây, ta tạm lưu theo thứ tự đọc được, sau này khi tính toán hiệu chỉnh
                    # hoặc khi đưa vào Madgwick, bạn cần đảm bảo đúng trục.
                    # Giả sử: mag_data_points lưu (mx, my, mz) theo hệ tọa độ cảm biến từ.
                    mag_data_points.append({'mag_x_raw': mx_raw, 'mag_y_raw': my_raw, 'mag_z_raw': mz_raw})
                    writer.writerow({'mag_x_raw': mx_raw, 'mag_y_raw': my_raw, 'mag_z_raw': mz_raw})
                    
                    # In ra để người dùng thấy dữ liệu đang được thu thập
                    if len(mag_data_points) % 20 == 0: # In mỗi 20 mẫu
                        print(f"Collected {len(mag_data_points)} samples: X={mx_raw}, Y={my_raw}, Z={mz_raw}")
                        
                except Exception as e:
                    print(f"Error reading magnetometer data: {e}")
                time.sleep(0.066) # Khoảng 15Hz
    except KeyboardInterrupt:
        print("Magnetometer data collection stopped by user.")
    
    print(f"Magnetometer data collection complete. Data saved to {output_file}")
    
    # Tính toán Hard-Iron offset cơ bản (tâm của đám mây điểm)
    if mag_data_points:
        mag_array = np.array([[p['mag_x_raw'], p['mag_y_raw'], p['mag_z_raw']] for p in mag_data_points])
        min_vals = np.min(mag_array, axis=0)
        max_vals = np.max(mag_array, axis=0)
        hard_iron_offset = (max_vals + min_vals) / 2.0
        print(f"Calculated Hard-Iron Offsets (approx): X={hard_iron_offset[0]:.2f}, Y={hard_iron_offset[1]:.2f}, Z={hard_iron_offset[2]:.2f}")
        print("For more accurate calibration (soft-iron), use specialized tools with the CSV data.")
        return {
            "hard_iron_offset_x": hard_iron_offset[0],
            "hard_iron_offset_y": hard_iron_offset[1],
            "hard_iron_offset_z": hard_iron_offset[2],
            "soft_iron_matrix": np.identity(3).tolist() # Placeholder, cần công cụ ngoài để tính
        }
    return None

# --- Lưu và Tải Tham số Hiệu chỉnh ---
def save_calibration_params(params, filename=CALIBRATION_FILE):
    try:
        with open(filename, 'w') as f:
            json.dump(params, f, indent=4)
        print(f"Calibration parameters saved to {filename}")
    except Exception as e:
        print(f"Error saving calibration parameters: {e}")

def load_calibration_params(filename=CALIBRATION_FILE):
    try:
        if os.path.exists(filename):
            with open(filename, 'r') as f:
                params = json.load(f)
            print(f"Calibration parameters loaded from {filename}")
            return params
        else:
            print(f"Calibration file {filename} not found. Using default/no calibration.")
    except Exception as e:
        print(f"Error loading calibration parameters: {e}")
    # Trả về một dict rỗng hoặc dict với giá trị mặc định nếu không load được
    return { 
        "gyro_bias_x": 0.0, "gyro_bias_y": 0.0, "gyro_bias_z": 0.0,
        "hard_iron_offset_x": 0.0, "hard_iron_offset_y": 0.0, "hard_iron_offset_z": 0.0,
        "soft_iron_matrix": np.identity(3).tolist() # Ma trận đơn vị (không hiệu chỉnh soft-iron)
    }


if __name__ == "__main__":
    # Khởi động MPU6050
    try:
        bus.write_byte_data(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00) # Wake MPU6050
        print("MPU6050 woken up.")
    except Exception as e:
        print(f"Could not wake MPU6050: {e}. Please check I2C connection.")
        sys.exit(1)

    calibration_data = load_calibration_params() # Tải hiệu chỉnh cũ nếu có

    while True:
        print("\nIMU Calibration Utility")
        print("1. Calibrate Gyroscope Bias")
        print("2. Collect Magnetometer Data (for offline calibration)")
        print("3. Save Current Calibration Parameters")
        print("4. View Current Calibration Parameters")
        print("5. Exit")
        choice = input("Enter your choice (1-5): ")

        if choice == '1':
            gyro_params = calibrate_gyro()
            if gyro_params:
                calibration_data.update(gyro_params)
                print("Gyro bias updated in current session. Remember to save.")
        elif choice == '2':
            mag_raw_file = f"mag_raw_data_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            mag_params = collect_magnetometer_data(duration_seconds=60, output_file=mag_raw_file) # Tăng thời gian để có nhiều dữ liệu
            if mag_params:
                # Chỉ lưu hard-iron offset từ tính toán đơn giản này
                # Soft-iron matrix cần công cụ chuyên dụng hơn để tính từ file CSV
                calibration_data["hard_iron_offset_x"] = mag_params["hard_iron_offset_x"]
                calibration_data["hard_iron_offset_y"] = mag_params["hard_iron_offset_y"]
                calibration_data["hard_iron_offset_z"] = mag_params["hard_iron_offset_z"]
                # calibration_data["soft_iron_matrix"] = mag_params["soft_iron_matrix"] # Giữ placeholder
                print(f"Magnetometer raw data saved to {mag_raw_file}.")
                print("Hard-iron offsets updated (approx). Use the CSV for full soft-iron calibration.")
                print("Remember to save all parameters (Choice 3).")

        elif choice == '3':
            save_calibration_params(calibration_data)
        elif choice == '4':
            print("\nCurrent Calibration Parameters:")
            print(json.dumps(calibration_data, indent=4))
        elif choice == '5':
            print("Exiting calibration utility.")
            break
        else:
            print("Invalid choice. Please try again.")
