import subprocess
import re
import numpy as np
import threading
import time

class RPLidar360:

    def __init__(self):
        self.bin_path = "/home/hthnh2/rplidar_sdk/output/Linux/Release/ultra_simple"
        self.scan = np.zeros(360)
        self.running = False
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

    def _reader(self):

        cmd = [
            self.bin_path,
            "--channel",
            "--serial",
            "/dev/ttyUSB0",
            "460800"
        ]

        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        pattern = re.compile(r"theta:\s*(\d+\.\d+)\s*Dist:\s*(\d+\.\d+)")

        temp_scan = np.zeros(360)

        while self.running:
            line = process.stdout.readline()
            match = pattern.search(line)

            if match:
                angle = int(float(match.group(1)))
                distance = float(match.group(2))

                if 0 <= angle < 360:
                    temp_scan[angle] = distance

                # nếu quay qua 359 → coi như 1 vòng xong
                if angle == 359:
                    with self.lock:
                        self.scan = temp_scan.copy()

        process.terminate()

    def get_scan(self):
        with self.lock:
            return self.scan.copy()

    def stop(self):
        self.running = False