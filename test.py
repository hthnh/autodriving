from ahrs.filters import Madgwick
import numpy as np
import math

print(f"AHRS library Madgwick class signature: {Madgwick.__init__.__doc__}")
# Khởi tạo với các tham số bạn nghĩ là đúng, ví dụ:
try:
    ahrs = Madgwick(sample_period=0.02, gain=0.1) # Thử với sample_period
    print("Initialized Madgwick with sample_period and gain.")
except TypeError:
    try:
        ahrs = Madgwick(frequency=50.0, gain=0.1) # Thử với frequency
        print("Initialized Madgwick with frequency and gain.")
    except TypeError:
        ahrs = Madgwick(gain=0.1) # Thử chỉ với gain
        print("Initialized Madgwick with only gain.")

print(f"Methods available for Madgwick object: {dir(ahrs)}")
if hasattr(ahrs, 'updateMARG'):
    print(f"updateMARG signature: {ahrs.updateMARG.__doc__}")
    print(f"updateMARG parameters: {ahrs.updateMARG.__code__.co_varnames}")
else:
    print("Method updateMARG not found directly. Check for 'update' or similar.")
    if hasattr(ahrs, 'update'):
         print(f"update signature: {ahrs.update.__doc__}")
         print(f"update parameters: {ahrs.update.__code__.co_varnames}")


# Dữ liệu giả định
q_prev = np.array([1.0, 0.0, 0.0, 0.0])
gyro = np.array([math.radians(1.0), math.radians(2.0), math.radians(3.0)])
accel = np.array([0.0, 0.0, 1.0])
mag = np.array([0.2, 0.0, 0.0])
dt_val = 0.02

try:
    print("Attempting to call updateMARG(q_old, gyr, acc, mag)...")
    # Dựa trên ví dụ của bạn: Q[t] = madgwick.updateMARG(Q[t-1], gyr=gyro_data[t], acc=acc_data[t], mag=mag_data[t])
    # Tên tham số quaternion cũ có thể là tham số vị trí đầu tiên, hoặc có tên là q, q0, q_old
    q_new = ahrs.updateMARG(q_prev, gyr=gyro, acc=accel, mag=mag) 
    print(f"updateMARG succeeded (without dt). New Q: {q_new}")
except TypeError as e:
    print(f"TypeError with updateMARG (without dt): {e}")
    try:
        print("Attempting to call updateMARG(q_old, gyr, acc, mag, dt)...")
        q_new = ahrs.updateMARG(q_prev, gyr=gyro, acc=accel, mag=mag, dt=dt_val)
        print(f"updateMARG succeeded (with dt). New Q: {q_new}")
    except TypeError as e2:
        print(f"TypeError with updateMARG (with dt): {e2}")
    except Exception as e_other:
        print(f"Other error with updateMARG (with dt): {e_other}")
except Exception as e_gen:
     print(f"General error with updateMARG: {e_gen}")