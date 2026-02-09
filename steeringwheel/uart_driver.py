# uart_driver.py
import serial
import time

class UARTDriver:
    def __init__(self, port="/dev/serial0", baudrate=115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.1
        )
        time.sleep(0.2)

    def send(self, data: bytes):
        self.ser.write(data)

    def read(self, size=1) -> bytes:
        return self.ser.read(size)

    def close(self):
        self.ser.close()
