import smbus
import time

ADDRESS = 0x10
bus = smbus.SMBus(1)

while True:
    try:
        # gửi lệnh đo 1 lần
        bus.write_byte_data(ADDRESS, 0x00, 0x04)

        time.sleep(0.05)

        data = bus.read_i2c_block_data(ADDRESS, 0x00, 9)

        if data[0] == 0x59 and data[1] == 0x59:
            distance = bus.read_word_data(ADDRESS, 0x00)
            print(distance)
        else:
            print("Invalid frame")

        time.sleep(0.1)

    except Exception as e:
        print("Error:", e)
        time.sleep(0.5)