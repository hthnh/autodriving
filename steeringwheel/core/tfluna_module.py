import smbus
import time

class TFLuna:
    def __init__(self, address=0x10, bus=1):
        self.bus = smbus.SMBus(bus)
        self.address = address

    def read_distance(self):
        data = self.bus.read_i2c_block_data(self.address, 0x00, 9)
        if data[0] == 0x59 and data[1] == 0x59:
            dist = data[2] + data[3] * 256
            return dist
        return None