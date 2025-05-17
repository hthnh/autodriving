import tkinter as tk
import serial
import time
import lgpio
import smbus2 # Use smbus2 for better compatibility
import math # Might be needed for sensor calculations


# --- Configuration ---
# !! MUST CHANGE THESE GPIO PINS TO MATCH YOUR WIRING (BCM numbering) !!
MOTOR_LEFT_FORWARD_PIN = 17
MOTOR_LEFT_BACKWARD_PIN = 27
MOTOR_RIGHT_FORWARD_PIN = 22
MOTOR_RIGHT_BACKWARD_PIN = 23
PWM_LEFT_PIN = 18  # Example PWM pin (check Pi's hardware PWM pins)
PWM_RIGHT_PIN = 19 # Example PWM pin (check Pi's hardware PWM pins)
PWM_FREQUENCY = 1000 # Hz for motor PWM
PWM_RANGE = 100 # Duty cycle range (0-100)

# I2C address
MPU6050_ADDR = 0x68
HMC5883L_ADDR = 0x1E
BMP_ADDR = 0x77

# MPU6050 Registers
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_GYRO_XOUT_H = 0x43


# HMC5883L Registers
HMC5883L_CONFIG_A = 0x00 # Config Register A
HMC5883L_CONFIG_B = 0x01 # Config Register B
HMC5883L_MODE_REG = 0x02 # Mode Register
HMC5883L_XOUT_H = 0x03   # Data Output X MSB Register
HMC5883L_ZOUT_H = 0x05   # Data Output Z MSB Register - Note: Z before Y
HMC5883L_YOUT_H = 0x07   # Data Output Y MSB Register

# BMP180 Registers
BMP180_CTRL_MEAS = 0xF4
BMP180_RESULT_MSB = 0xF6
BMP180_RESULT_LSB = 0xF7
BMP180_RESULT_XLSB= 0xF8 # Only used for higher resolution pressure

BMP180_CMD_READ_TEMP = 0x2E
BMP180_CMD_READ_PRESSURE_0 = 0x34 # oss=0
BMP180_CMD_READ_PRESSURE_1 = 0x74 # oss=1
BMP180_CMD_READ_PRESSURE_2 = 0xB4 # oss=2
BMP180_CMD_READ_PRESSURE_3 = 0xF4 # oss=3

# Choose pressure oversampling setting (0, 1, 2, 3) - higher means more accuracy but slower
BMP180_OSS = 0 # Using standard resolution for faster readings

# Global variables for calibration
calibration = {}

bus = smbus2.SMBus(1)

# Wake up MPU6050
bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0)

# HMC5883L continuous mode
bus.write_byte_data(HMC5883L_ADDR, HMC5883L_MODE_REG, 0x00)

# Serial Port for Arduino Nano
SERIAL_PORT = '/dev/ttyAMA0' # Or '/dev/ttyUSB0', etc.
SERIAL_BAUDRATE = 9600

# UI Scaling
UI_SCALE_FACTOR = 1.25
# --- End Configuration ---



class CarControlApp:
    def __init__(self, master):
        self.master = master
        master.title("RPi Car Control (NRF Remote)") # ?i tiu d?

        self.h = None # Handle for lgpio
        self.bus = None # Handle for smbus2
        self.bmp180_calibration = None
        self.last_heading = 0.0

        # --------- 1. Initialize lgpio for Motor Control --------
        # <<< Gi? nguyn ph?n kh?i t?o lgpio >>>
        try:
            self.h = lgpio.gpiochip_open(0) # Open default GPIO chip
            if self.h >= 0:
                print("lgpio connected successfully.")
                # Define motor control pins
                self.motor_left_forward = MOTOR_LEFT_FORWARD_PIN
                self.motor_left_backward = MOTOR_LEFT_BACKWARD_PIN
                self.motor_right_forward = MOTOR_RIGHT_FORWARD_PIN
                self.motor_right_backward = MOTOR_RIGHT_BACKWARD_PIN
                self.pwm_left = PWM_LEFT_PIN
                self.pwm_right = PWM_RIGHT_PIN
                self.pwm_range = PWM_RANGE
                self.pwm_frequency = PWM_FREQUENCY

                # Claim GPIO pins as outputs
                lgpio.gpio_claim_output(self.h, self.motor_left_forward)
                lgpio.gpio_claim_output(self.h, self.motor_left_backward)
                lgpio.gpio_claim_output(self.h, self.motor_right_forward)
                lgpio.gpio_claim_output(self.h, self.motor_right_backward)

                # Claim and configure PWM pins using tx_pwm for hardware PWM
                lgpio.gpio_claim_output(self.h, self.pwm_left)
                lgpio.gpio_claim_output(self.h, self.pwm_right)
                # Start PWM with 0% duty cycle
                lgpio.tx_pwm(self.h, self.pwm_left, self.pwm_frequency, 0)
                lgpio.tx_pwm(self.h, self.pwm_right, self.pwm_frequency, 0)
                print(f"PWM configured on GPIO {self.pwm_left} and {self.pwm_right} at {self.pwm_frequency} Hz.")

                # Initial motor state: stopped
                self.stop_cmd(log=False) # Call stop initially

            else:
                print(f"Error opening lgpio chip: handle={self.h}")
                self.h = None # Ensure handle is None if failed
        except ImportError:
            print("Error: lgpio library not found. Install it (pip install lgpio)")
            print("Ensure the lg daemon is running ('sudo systemctl start lg')")
            self.h = None
        except Exception as e:
            print(f"Error initializing lgpio: {e}")
            self.h = None


        # ------ 2. Initialize Serial Port for Control Data -------
        self.ser = None
        try:
            # Tang timeout n?u c?n, v chng ta d?c s? byte c? d?nh
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.1)
            print(f"Serial port {SERIAL_PORT} opened successfully for control data.")
            # G?i hm d?c d? li?u di?u khi?n m?i
            self.master.after(50, self.read_control_data) # Ki?m tra d? li?u thu?ng xuyn
        except serial.SerialException as e:
            print(f"Error opening serial port {SERIAL_PORT}: {e}")
            print("Check connection, permissions, and if Arduino receiver is running.")
            self.ser = None
        except Exception as e:
            print(f"An unexpected error occurred during serial setup: {e}")
            self.ser = None

        # ------ 3. Initialize I2C for GY-87 --------
        # <<< Gi? nguyn ph?n kh?i t?o I2C v GY-87 >>>
        self.gy87_data = {} # Store sensor readings
        try:
            self.bus = smbus2.SMBus(1) # Use I2C bus 1 on Raspberry Pi
            print("I2C bus 1 opened successfully.")

            # a) Wake up MPU6050
            try:
                self.bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0)
                print("MPU6050 woken up.")
            except OSError as e:
                 print(f"Warning: Could not write to MPU6050 ({hex(MPU6050_ADDR)}): {e}.")

            # b) Configure HMC5883L (set continuous measurement mode)
            try:
                # Set to continuous measurement mode (0x00)
                self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_MODE_REG, 0x00)
                # Optionally configure gain, data rate in CONFIG_A, CONFIG_B
                print("HMC5883L set to continuous mode.")
            except OSError as e:
                 print(f"Warning: Could not write to HMC5883L ({hex(HMC5883L_ADDR)}): {e}.")

            # c) Read BMP180 Calibration Data
            try:
                self.bmp180_calibration = self._bmp180_read_calibration()
                if self.bmp180_calibration:
                    print("BMP180 calibration data read successfully.")
                else:
                    print("Warning: Failed to read BMP180 calibration data.")

            except OSError as e:
                 print(f"Warning: Could not read BMP180 ({hex(BMP_ADDR)}) calibration: {e}.")
                 self.bmp180_calibration = None # Ensure it's None if failed

            # Start reading GY-87 data periodically
            if self.bus: # Ch? d?c n?u I2C ho?t d?ng
                self.master.after(200, self.update_gy87_data_ui) # Start periodic reads/updates

        except FileNotFoundError:
            print("Error: I2C bus 1 not found. Is I2C enabled in raspi-config?")
            self.bus = None
        except Exception as e:
            print(f"Error initializing I2C smbus: {e}")
            self.bus = None

        # --- Scaling the interface ---
        # <<< Gi? nguyn >>>
        try:
            self.master.tk.call('tk', 'scaling', UI_SCALE_FACTOR)
        except tk.TclError:
            print("Note: Could not set UI scaling (might not be supported on this OS/Tk version).")

        # --- UI Elements ---
        self.create_ui() # Call AFTER initializing components

        # --- KHNG cn c?p nh?t hi?n th? c?m bi?n Nano ---
        # self.master.after(100, self.update_nano_sensor_display) # XA DG NY

        # --- Bind Canvas Resize ---
        # self.sensor_canvas.bind("<Configure>", self.redraw_canvas) # XA ho?c s?a n?u canvas cn ng

        # --- Set close protocol ---
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)


    # --- I2C Helper Functions ---
    # <<< Gi? nguyn cc hm _read_signed_byte, _read_unsigned_byte, etc. >>>
    def _read_signed_byte(self, addr, reg):
        """Reads a signed byte from I2C."""
        try:
            val = self.bus.read_byte_data(addr, reg)
            return val if val < 128 else val - 256
        except OSError as e:
            # print(f"Warning: I2C signed byte read error from addr {hex(addr)}, reg {hex(reg)}: {e}")
            return None

    def _read_unsigned_byte(self, addr, reg):
        """Reads an unsigned byte from I2C."""
        try:
            return self.bus.read_byte_data(addr, reg)
        except OSError as e:
            # print(f"Warning: I2C unsigned byte read error from addr {hex(addr)}, reg {hex(reg)}: {e}")
            return None

    def _read_signed_word_2c(self, addr, reg):
        """Reads a signed 16-bit word (big-endian) using 2's complement."""
        try:
            high = self.bus.read_byte_data(addr, reg)
            low = self.bus.read_byte_data(addr, reg + 1)
            val = (high << 8) + low
            # S?a l?i nh?: M?c d?nh hm ny d?c Big Endian, nhung n?u c?m bi?n tr? v? Little Endian th c?n d?o l?i
            # Tuy nhin, MPU6050, HMC5883L thu?ng  Big Endian. BMP180 cung v?y khi d?c t?ng byte.
            # Gi? nguy logic ny cho cc hm d?c sensor hi?n c.
            return val if val < 32768 else val - 65536
        except OSError as e:
            # print(f"Warning: I2C signed word read error from addr {hex(addr)}, reg {hex(reg)}: {e}")
            return None

    def _read_unsigned_word(self, addr, reg):
        """Reads an unsigned 16-bit word (big-endian)."""
        try:
            high = self.bus.read_byte_data(addr, reg)
            low = self.bus.read_byte_data(addr, reg + 1)
            return (high << 8) + low
        except OSError as e:
            # print(f"Warning: I2C unsigned word read error from addr {hex(addr)}, reg {hex(reg)}: {e}")
            return None

    # --- BMP180 Specific Functions ---
    # <<< Gi? nguyn cc hm BMP180 >>>
    def _bmp180_read_calibration(self):
        """Reads the factory calibration data from the BMP180."""
        if not self.bus: return None
        cal = {}
        # S? d?ng _read_signed_word_2c v _read_unsigned_word nhu cu v BMP180 tr? v? Big Endian
        cal['AC1'] = self._read_signed_word_2c(BMP_ADDR, 0xAA)
        cal['AC2'] = self._read_signed_word_2c(BMP_ADDR, 0xAC)
        cal['AC3'] = self._read_signed_word_2c(BMP_ADDR, 0xAE)
        cal['AC4'] = self._read_unsigned_word(BMP_ADDR, 0xB0)
        cal['AC5'] = self._read_unsigned_word(BMP_ADDR, 0xB2)
        cal['AC6'] = self._read_unsigned_word(BMP_ADDR, 0xB4)
        cal['B1'] = self._read_signed_word_2c(BMP_ADDR, 0xB6)
        cal['B2'] = self._read_signed_word_2c(BMP_ADDR, 0xB8)
        cal['MB'] = self._read_signed_word_2c(BMP_ADDR, 0xBA)
        cal['MC'] = self._read_signed_word_2c(BMP_ADDR, 0xBC)
        cal['MD'] = self._read_signed_word_2c(BMP_ADDR, 0xBE)

        # Check if any calibration value failed to read
        if None in cal.values():
            print("Error: Failed to read one or more BMP180 calibration values.")
            return None
        return cal

    def _bmp180_read_raw_temp(self):
        """Sends temp command and reads raw temperature value."""
        if not self.bus: return None
        try:
            self.bus.write_byte_data(BMP_ADDR, BMP180_CTRL_MEAS, BMP180_CMD_READ_TEMP)
            time.sleep(0.005) # Wait for conversion (datasheet: max 4.5ms)
            ut = self._read_unsigned_word(BMP_ADDR, BMP180_RESULT_MSB) # ?c Big Endian
            return ut
        except OSError as e:
            # print(f"Warning: BMP180 raw temp read error: {e}")
            return None

    def _bmp180_read_raw_pressure(self):
        """Sends pressure command and reads raw pressure value."""
        if not self.bus: return None
        try:
            # Select command based on oversampling setting
            cmd = BMP180_CMD_READ_PRESSURE_0 + (BMP180_OSS << 6)
            # Delay based on OSS (datasheet: 4.5, 7.5, 13.5, 25.5 ms)
            delays = [0.005, 0.008, 0.014, 0.026]
            self.bus.write_byte_data(BMP_ADDR, BMP180_CTRL_MEAS, cmd)
            time.sleep(delays[BMP180_OSS])

            # Read MSB, LSB, XLSB
            msb = self._read_unsigned_byte(BMP_ADDR, BMP180_RESULT_MSB)
            lsb = self._read_unsigned_byte(BMP_ADDR, BMP180_RESULT_LSB)
            xlsb = self._read_unsigned_byte(BMP_ADDR, BMP180_RESULT_XLSB)
            if None in [msb, lsb, xlsb]: return None # Check if read failed

            up = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - BMP180_OSS)
            return up
        except OSError as e:
            # print(f"Warning: BMP180 raw pressure read error: {e}")
            return None

    def _bmp180_calculate_temp_pressure(self, ut, up):
        """Calculates true temp (C) and pressure (Pa) from raw values and calibration."""
        if self.bmp180_calibration is None or ut is None or up is None:
            return None, None
        try:
            cal = self.bmp180_calibration
            # Calculate true temperature
            X1 = ((ut - cal['AC6']) * cal['AC5']) >> 15
            # Thm ki?m tra X1 + cal['MD'] khc 0
            if (X1 + cal['MD']) == 0: return None, None
            X2 = (cal['MC'] << 11) // (X1 + cal['MD']) # Use // for integer division
            B5 = X1 + X2
            temp_c = ((B5 + 8) >> 4) / 10.0

            # Calculate true pressure
            B6 = B5 - 4000
            X1 = (cal['B2'] * ((B6 * B6) >> 12)) >> 11
            X2 = (cal['AC2'] * B6) >> 11
            X3 = X1 + X2
            # Thm ki?m tra AC1 c t?n t?i khng
            if 'AC1' not in cal: return temp_c, None
            B3 = (((cal['AC1'] * 4 + X3) << BMP180_OSS) + 2) >> 2 # Shift left by OSS
            X1 = (cal['AC3'] * B6) >> 13
            X2 = (cal['B1'] * ((B6 * B6) >> 12)) >> 16
            X3 = ((X1 + X2) + 2) >> 2
            B4 = (cal['AC4'] * (X3 + 32768)) >> 15
            # Avoid division by zero
            if B4 == 0: return temp_c, None
            B7 = (up - B3) * (50000 >> BMP180_OSS)
            if B7 < 0x80000000:
                 p = (B7 * 2) // B4
            else:
                 p = (B7 // B4) * 2
            X1 = (p >> 8) * (p >> 8)
            X1 = (X1 * 3038) >> 16
            X2 = (-7357 * p) >> 16
            pressure_pa = p + ((X1 + X2 + 3791) >> 4)

            return temp_c, pressure_pa
        except (TypeError, ZeroDivisionError, KeyError) as e:
             print(f"Error during BMP180 calculation: {e}")
             return None, None

    def read_accel_data(self):
        """Reads accelerometer data and converts to g's."""
        if not self.bus: return None, None, None
 
        ax_r = self._read_signed_word_2c(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H)
        ay_r = self._read_signed_word_2c(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H + 2)
        az_r = self._read_signed_word_2c(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H + 4)
        if None in [ax_r, ay_r, az_r]: return None, None, None

        # Convert to g (assuming default +/- 2g scale)
        # Sensitivity = 16384 LSB/g
        ax_g = ax_r / 16384.0
        ay_g = ay_r / 16384.0
        az_g = az_r / 16384.0
        return ax_g, ay_g, az_g

    def read_gyro_data(self):
        """Reads gyroscope data and converts to degrees/second."""
        if not self.bus: return None, None, None
        gx_r = self._read_signed_word_2c(MPU6050_ADDR, MPU6050_GYRO_XOUT_H)
        gy_r = self._read_signed_word_2c(MPU6050_ADDR, MPU6050_GYRO_XOUT_H + 2)
        gz_r = self._read_signed_word_2c(MPU6050_ADDR, MPU6050_GYRO_XOUT_H + 4)
        if None in [gx_r, gy_r, gz_r]: return None, None, None

        # Convert to dps (assuming default +/- 250 dps scale)
        # Sensitivity = 131 LSB/(deg/s)
        gx_dps = gx_r / 131.0
        gy_dps = gy_r / 131.0
        gz_dps = gz_r / 131.0
        return gx_dps, gy_dps, gz_dps

    # --- HMC5883L Specific Functions ---
    # <<< Gi? nguyn cc hm HMC5883L >>>
    def read_mag_data(self):
        """Reads magnetometer data (raw) and calculates heading."""
        if not self.bus: return None, None, None, None
        # Note: HMC5883L output order is X, Z, Y
        # HMC5883L cung tr? v? Big Endian
        mx_r = self._read_signed_word_2c(HMC5883L_ADDR, HMC5883L_XOUT_H)
        mz_r = self._read_signed_word_2c(HMC5883L_ADDR, HMC5883L_ZOUT_H)
        my_r = self._read_signed_word_2c(HMC5883L_ADDR, HMC5883L_YOUT_H)
        if None in [mx_r, my_r, mz_r]: return None, None, None, None

        # Calculate heading (simple, assumes sensor is level)
        try:
            heading_rad = math.atan2(my_r, mx_r) # S? d?ng gi tr? th
            heading_deg = math.degrees(heading_rad)
            # Normalize to 0-360 degrees
            if heading_deg < 0:
                heading_deg += 360
            self.last_heading = heading_deg # Store for potential filtering later
        except Exception as e:
            print(f"Error calculating heading: {e}")
            heading_deg = self.last_heading # Return last known value on error

        # Return raw values and calculated heading
        return mx_r, my_r, mz_r, heading_deg

    # --- Combined GY-87 Data Reading ---
    # <<< Gi? nguyn hm read_gy87_data >>>
    def read_gy87_data(self):
        """Reads and processes data from all GY-87 components."""
        results = {
            "accel_g": {'x': None, 'y': None, 'z': None},
            "gyro_dps": {'x': None, 'y': None, 'z': None},
            "mag_raw": {'x': None, 'y': None, 'z': None},
            "heading": None,
            "temp_c": None,
            "pressure_pa": None
        }
        if not self.bus:
            return results # Return Nones if bus not available

        # 1. Read Accel
        ax_g, ay_g, az_g = self.read_accel_data()
        results["accel_g"] = {'x': ax_g, 'y': ay_g, 'z': az_g}

        # 2. Read Gyro
        gx_dps, gy_dps, gz_dps = self.read_gyro_data()
        results["gyro_dps"] = {'x': gx_dps, 'y': gy_dps, 'z': gz_dps}

        # 3. Read Magnetometer
        mx_r, my_r, mz_r, heading = self.read_mag_data()
        results["mag_raw"] = {'x': mx_r, 'y': my_r, 'z': mz_r}
        results["heading"] = heading

        # 4. Read BMP180 Temp/Pressure
        if self.bmp180_calibration:
            ut = self._bmp180_read_raw_temp()
            up = self._bmp180_read_raw_pressure()
            temp, pressure = self._bmp180_calculate_temp_pressure(ut, up)
            results["temp_c"] = temp
            results["pressure_pa"] = pressure
        else:
            results["temp_c"] = None
            results["pressure_pa"] = None

        return results

    def create_ui(self):
   
        self.gy87_frame = tk.Frame(self.master, bd=2, relief=tk.SUNKEN)
        self.gy87_frame.grid(row=0, column=0, columnspan=3, padx=10, pady=10, sticky="ew") # ?i row=0
        tk.Label(self.gy87_frame, text="GY-87 Processed Data", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=2, pady=(2,5))

        self.gy87_labels = {}
        gy87_data_info = {
            "Accel (g) X:": "accel_g_x", "Y:": "accel_g_y", "Z:": "accel_g_z",
            "Gyro (/s) X:": "gyro_dps_x", "Y:": "gyro_dps_y", "Z:": "gyro_dps_z",
            "Heading ():": "heading",
            "Temp (C):": "temp_c",
            "Pressure (Pa):": "pressure_pa",
            "Mag Raw X:": "mag_raw_x", "Y:": "mag_raw_y", "Z:": "mag_raw_z",
        }
        row_num = 1
        for label_text, data_key in gy87_data_info.items():
            desc_label = tk.Label(self.gy87_frame, text=label_text, anchor='w', justify=tk.LEFT)
            desc_label.grid(row=row_num, column=0, padx=(10, 2), pady=1, sticky="w")
            value_label = tk.Label(self.gy87_frame, text="N/A", anchor='w', justify=tk.LEFT, width=15)
            value_label.grid(row=row_num, column=1, padx=(2, 10), pady=1, sticky="ew")
            self.gy87_labels[data_key] = value_label
            row_num += 1
        self.gy87_frame.grid_columnconfigure(1, weight=1)

   
        control_frame = tk.Frame(self.master)
        control_frame.grid(row=1, column=0, columnspan=3, padx=10, pady=10) # ?i row=1
        btn_width = 10
       
        self.forward_btn = tk.Button(control_frame, text="Forward (W)", width=btn_width, command=self.move_forward_cmd) # B? command
        self.backward_btn = tk.Button(control_frame, text="Backward (S)", width=btn_width, command=self.move_backward_cmd) # B? command
        self.left_btn = tk.Button(control_frame, text="Left (A)", width=btn_width, command=self.turn_left_cmd) # B? command
        self.right_btn = tk.Button(control_frame, text="Right (D)", width=btn_width, command=self.turn_right_cmd) # B? command
        self.stop_btn = tk.Button(control_frame, text="STOP (Space)", width=btn_width, command=self.stop_cmd, bg="red", fg="white") 
        self.forward_btn.grid(row=0, column=1, padx=5, pady=5)
        self.left_btn.grid(row=1, column=0, padx=5, pady=5)
        self.stop_btn.grid(row=1, column=1, padx=5, pady=5)
        self.right_btn.grid(row=1, column=2, padx=5, pady=5)
        self.backward_btn.grid(row=2, column=1, padx=5, pady=5)


        self.master.bind('<w>', lambda event: self.move_forward_cmd())
        self.master.bind('<s>', lambda event: self.move_backward_cmd())
        self.master.bind('<a>', lambda event: self.turn_left_cmd())
        self.master.bind('<d>', lambda event: self.turn_right_cmd())
        self.master.bind('<space>', lambda event: self.stop_cmd())
        self.master.bind('<KeyRelease-w>', lambda event: self.stop_cmd()) 
        self.master.bind('<KeyRelease-s>', lambda event: self.stop_cmd())
        self.master.bind('<KeyRelease-a>', lambda event: self.stop_cmd())
        self.master.bind('<KeyRelease-d>', lambda event: self.stop_cmd())
        
        
    def update_gy87_data_ui(self):
        """Fetches processed GY-87 data and updates the UI labels."""
        if not self.bus:
          
             self.master.after(1000, self.update_gy87_data_ui) # Check again later
             return

        # Fetch the latest processed data
        current_data = self.read_gy87_data()

        acc_x = current_data["accel_g"]['x']
        acc_y = current_data["accel_g"]['y']
        acc_z = current_data["accel_g"]['z']
        if "accel_g_x" in self.gy87_labels: self.gy87_labels["accel_g_x"].config(text=f"{acc_x:.2f}" if acc_x is not None else "N/A")
        if "accel_g_y" in self.gy87_labels: self.gy87_labels["accel_g_y"].config(text=f"{acc_y:.2f}" if acc_y is not None else "N/A")
        if "accel_g_z" in self.gy87_labels: self.gy87_labels["accel_g_z"].config(text=f"{acc_z:.2f}" if acc_z is not None else "N/A")

        # --- Update Gyro Labels ---
        gyr_x = current_data["gyro_dps"]['x']
        gyr_y = current_data["gyro_dps"]['y']
        gyr_z = current_data["gyro_dps"]['z']
        if "gyro_dps_x" in self.gy87_labels: self.gy87_labels["gyro_dps_x"].config(text=f"{gyr_x:.2f}" if gyr_x is not None else "N/A")
        if "gyro_dps_y" in self.gy87_labels: self.gy87_labels["gyro_dps_y"].config(text=f"{gyr_y:.2f}" if gyr_y is not None else "N/A")
        if "gyro_dps_z" in self.gy87_labels: self.gy87_labels["gyro_dps_z"].config(text=f"{gyr_z:.2f}" if gyr_z is not None else "N/A")

        # --- Update Heading Label ---
        head = current_data["heading"]
        if "heading" in self.gy87_labels: self.gy87_labels["heading"].config(text=f"{head:.1f}" if head is not None else "N/A")

        # --- Update Temp Label ---
        temp = current_data["temp_c"]
        if "temp_c" in self.gy87_labels: self.gy87_labels["temp_c"].config(text=f"{temp:.1f}" if temp is not None else "N/A")

        # --- Update Pressure Label ---
        pres = current_data["pressure_pa"]
        if "pressure_pa" in self.gy87_labels: self.gy87_labels["pressure_pa"].config(text=f"{pres:.0f}" if pres is not None else "N/A")

        # --- Update Raw Mag Labels (Optional) ---
        mag_x = current_data["mag_raw"]['x']
        mag_y = current_data["mag_raw"]['y']
        mag_z = current_data["mag_raw"]['z']
        if "mag_raw_x" in self.gy87_labels: self.gy87_labels["mag_raw_x"].config(text=f"{mag_x}" if mag_x is not None else "N/A")
        if "mag_raw_y" in self.gy87_labels: self.gy87_labels["mag_raw_y"].config(text=f"{mag_y}" if mag_y is not None else "N/A")
        if "mag_raw_z" in self.gy87_labels: self.gy87_labels["mag_raw_z"].config(text=f"{mag_z}" if mag_z is not None else "N/A")


        for key, label in self.gy87_labels.items():
             try:
                 pass # label.config(...) done above
             except tk.TclError:
                 print(f"Warning: Label for {key} no longer exists.")
                 pass # Ignore if widget is destroyed

        # Schedule next update
        self.master.after(500, self.update_gy87_data_ui) # Update ~twice per second


    def read_control_data(self):
        """Reads control data string from Serial, parses it, and triggers actions."""
        line = None 
        if self.ser and self.ser.is_open:
            try:
               
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                    if line: # Ch? x? l n?u d?c du?c dng khng r?ng
                        # print(f"Raw Serial: {line}") # Debug

                        # --- Phn th chu?i ---
                        if not ("Buttons:" in line and "Joy1(X,Y):" in line and "Joy2(X,Y):" in line):
                            print(f"Warning: Malformed line (missing keywords): {line}")
                            # B? qua g l?i y v ch? dng ti?p theo
                            return # Thot kh?i try block nhung v?n schedule l?n d?c ti?p theo ? finally

                        parts = line.split('|')
                        if len(parts) != 3:
                            print(f"Warning: Malformed line (expected 3 parts separated by '|'): {line}")
                            return

                        # 1. Buttons
                        button_part_str = parts[0].split(':')[1].strip()
                        button_strings = button_part_str.split()
                        if len(button_strings) != 11:
                            print(f"Warning: Malformed line (expected 11 buttons): {line}")
                            return
                        # Chuy?n thnh list cc boolean (True n?u l '1', False n?u l '0')
                        buttons = [s == '1' for s in button_strings]

                        # 2. Joy1
                        joy1_part_str = parts[1].split(':')[1].strip()
                        joy1_strings = joy1_part_str.split(',')
                        if len(joy1_strings) != 2:
                            print(f"Warning: Malformed line (expected 2 values for Joy1): {line}")
                            return
                        joy1_x = int(joy1_strings[0].strip())
                        joy1_y = int(joy1_strings[1].strip())

                        # 3. Joy2
                        joy2_part_str = parts[2].split(':')[1].strip()
                        joy2_strings = joy2_part_str.split(',')
                        if len(joy2_strings) != 2:
                            print(f"Warning: Malformed line (expected 2 values for Joy2): {line}")
                            return
                        joy2_x = int(joy2_strings[0].strip())
                        joy2_y = int(joy2_strings[1].strip())

                        # --- Ki?m tra gi tr? Joystick (0-10) ---
                        if not (0 <= joy1_x <= 10 and 0 <= joy1_y <= 10 and \
                                0 <= joy2_x <= 10 and 0 <= joy2_y <= 10):
                            print(f"Warning: Joystick values out of range (0-10): {line}")
                            return # B? qua dng c gi tr? l?i

                        # --- Debug print (ty ch?n) ---
                        print(f"Parsed: Btns={buttons}, J1=({joy1_x},{joy1_y}), J2=({joy2_x},{joy2_y})")

                        
                        self.control_with_joystick(joy1_x, joy1_y)

                      
                        if buttons[6]:
                             print("Remote Emergency Stop Pressed!")
                             self.stop_cmd()



            except serial.SerialException as e:
                print(f"Serial Error during read: {e}")
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = None
                print("Serial reading stopped due to error.")
                return # Stop trying to read
            except (ValueError, IndexError) as e:
                # L?i khi chuy?n d?i int() ho?c truy c?p index list/chu?i khng h?p l?
                print(f"Error parsing line: {e} - Line: {line}")
                # C th? xa b? d?m n?u d? li?u lin t?c l?i:
                # if self.ser and self.ser.is_open: self.ser.reset_input_buffer()
            except Exception as e:
                 print(f"Unexpected error reading control data: {e}")

        # Ln l?ch d?c l?i sau m?t kho?ng th?i gian ng?n
            finally:
                if self.master: # Ki?m tra c?a s? chnh cn t?n t?i
                    # N?u khg d?c du?c line no (timeout ho?c chua cd? li?u), v?n l l?ch d?c l?i
                    self.master.after(30, self.read_control_data) 



             

   
    def map_value(self, x, in_min, in_max, out_min, out_max):
      
        if in_max == in_min:
            return out_min
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    
    def move_forward_cmd(self, speed=100):
        if self.h is not None:
            # print(f"CMD: Move Forward (Speed: {speed}%)")
            self._set_motor_direction(1, 0, 1, 0)
            duty_cycle = max(0, min(100, speed)) # Gi?i h?n t?c d? 0-100
            self._set_motor_speed(duty_cycle, duty_cycle)

    def move_backward_cmd(self, speed=100): # 
        if self.h is not None:
            # print(f"CMD: Move Backward (Speed: {speed}%)")
            self._set_motor_direction(0, 1, 0, 1)
            duty_cycle = max(0, min(100, speed))
            self._set_motor_speed(duty_cycle, duty_cycle)


    def turn_left_cmd(self, speed=80):
        if self.h is not None:
            # print(f"CMD: Turn Left (Speed: {speed}%)")
            self._set_motor_direction(0, 1, 1, 0) # Pivot turn: Left back, Right fwd
            duty_cycle = max(0, min(100, speed))
            self._set_motor_speed(duty_cycle, duty_cycle)

    def turn_right_cmd(self, speed=80):
         if self.h is not None:
            # print(f"CMD: Turn Right (Speed: {speed}%)")
            self._set_motor_direction(1, 0, 0, 1) # Pivot turn: Left fwd, Right back
            duty_cycle = max(0, min(100, speed))
            self._set_motor_speed(duty_cycle, duty_cycle)


    def control_with_joystick(self, joy_x, joy_y):
        
         if self.h is None: return

         throttle = int((joy_y - 5) * 20)  # (5-5)*20=0, (10-5)*20=100, (0-5)*20=-100
         steering = int((joy_x - 5) * 20)  # (5-5)*20=0, (10-5)*20=100, (0-5)*20=-100

         left_speed = throttle + steering
         right_speed = throttle - steering

         # Gi?i h?n t?c d? trong kho?ng -100 d?n 100
         left_speed = max(-100, min(100, left_speed))
         right_speed = max(-100, min(100, right_speed))

         if left_speed >= 0:
             lgpio.gpio_write(self.h, self.motor_left_forward, 1)
             lgpio.gpio_write(self.h, self.motor_left_backward, 0)
             # Chuy?n d?i t?c d? (-100->100) thnh duty cycle (0->PWM_RANGE)
             left_pwm = int(left_speed * self.pwm_range / 100.0)
         else: # left_speed < 0
             lgpio.gpio_write(self.h, self.motor_left_forward, 0)
             lgpio.gpio_write(self.h, self.motor_left_backward, 1)
             left_pwm = int(-left_speed * self.pwm_range / 100.0) # PWM l gi tr? duong


         # Bnh ph?i
         if right_speed >= 0:
             lgpio.gpio_write(self.h, self.motor_right_forward, 1)
             lgpio.gpio_write(self.h, self.motor_right_backward, 0)
             right_pwm = int(right_speed * self.pwm_range / 100.0)
         else: # right_speed < 0
             lgpio.gpio_write(self.h, self.motor_right_forward, 0)
             lgpio.gpio_write(self.h, self.motor_right_backward, 1)
             right_pwm = int(-right_speed * self.pwm_range / 100.0) # PWM l gi tr? duong

         # Gi?i h?n PWM trong kho?ng 0 d?n PWM_RANGE
         left_pwm = max(0, min(self.pwm_range, left_pwm))
         right_pwm = max(0, min(self.pwm_range, right_pwm))

         # --- t PWM ---
         # S? d?ng hm _set_motor_speed d c
         self._set_motor_speed(left_pwm, right_pwm)
         # print(f"Control: T={throttle}, S={steering} => L={left_speed}({left_pwm}), R={right_speed}({right_pwm})") # Debug


    def stop_cmd(self, log=True):
        if self.h is not None:
            if log: print("CMD: Stop")
            self._set_motor_direction(0, 0, 0, 0)
            self._set_motor_speed(0, 0)

    def _set_motor_speed(self, left_duty, right_duty):
        """Sets PWM duty cycle for left and right motors."""
        if self.h is None: return
        left_duty = max(0, min(self.pwm_range, left_duty))
        right_duty = max(0, min(self.pwm_range, right_duty))
        try:
            lgpio.tx_pwm(self.h, self.pwm_left, self.pwm_frequency, left_duty)
            lgpio.tx_pwm(self.h, self.pwm_right, self.pwm_frequency, right_duty)
        except Exception as e:
            print(f"Error setting PWM: {e}")

    def _set_motor_direction(self, left_fwd, left_bwd, right_fwd, right_bwd):
        """Sets direction pins for L298N."""
        if self.h is None: return
        try:
            lgpio.gpio_write(self.h, self.motor_left_forward, left_fwd)
            lgpio.gpio_write(self.h, self.motor_left_backward, left_bwd)
            lgpio.gpio_write(self.h, self.motor_right_forward, right_fwd)
            lgpio.gpio_write(self.h, self.motor_right_backward, right_bwd)
        except Exception as e:
            print(f"Error setting motor direction: {e}")


    def on_closing(self):
        """Cleanup resources before closing the application."""
        print("Closing application and cleaning up...")
        # Stop motors first
        self.stop_cmd(log=False)
        time.sleep(0.1)


        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print("Serial port closed.")
            except Exception as e:
                 print(f"Error closing serial port: {e}")

        # Close I2C bus
        if self.bus:
             try:
                 self.bus.close()
                 print("I2C bus closed.")
             except Exception as e:
                  print(f"Error closing I2C bus: {e}")

        # Release lgpio resources
        if self.h is not None:
            print("Releasing lgpio resources...")
            try:
                # Stop PWM explicitly
                lgpio.tx_pwm(self.h, self.pwm_left, self.pwm_frequency, 0)
                lgpio.tx_pwm(self.h, self.pwm_right, self.pwm_frequency, 0)
                time.sleep(0.05)
                # Free GPIO pins
          
                lgpio.gpio_free(self.h, self.motor_left_forward)
                lgpio.gpio_free(self.h, self.motor_left_backward)
                lgpio.gpio_free(self.h, self.motor_right_forward)
                lgpio.gpio_free(self.h, self.motor_right_backward)
                lgpio.gpio_free(self.h, self.pwm_left)
                lgpio.gpio_free(self.h, self.pwm_right)

                # Close the GPIO chip handle
                lgpio.gpiochip_close(self.h)
                print("lgpio resources released.")
            except Exception as e:
                print(f"Error releasing lgpio resources: {e}")

        # Destroy the Tkinter window
        if self.master:
            self.master.destroy()
            print("Tkinter window destroyed.")
        print("Application closed.")
        
        
        
if __name__ == "__main__":
    # --- Pre-run Checks/Instructions (C?p nh?t) ---
    print("--- RPi Car Control (NRF Remote) ---")
    print("Ensure:")
    print(" 1. Raspberry Pi is configured (I2C enabled).")
    print(" 2. Required libraries: pip install pyserial lgpio smbus2")
    print(" 3. 'lg' daemon is running: sudo systemctl start lg")
    print(f" 4. Arduino Nano (NRF Receiver) is connected to {SERIAL_PORT} and sending control struct data.")
    print(f" 5. GY-87 sensor is connected to I2C bus 1 (optional).")
    print(f" 6. L298N motor driver connections are correct (check BCM pins).")
    print(" 7. Correct power is supplied.")
   
    print("-------------------------")

    root = tk.Tk()
    app = CarControlApp(root)
    root.mainloop()
