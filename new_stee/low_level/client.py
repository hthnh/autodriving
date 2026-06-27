import time
import re
import threading
import serial

from control.intent import ControlIntent


HEADER = 0xAA

DIR_STOP = 0
DIR_FORWARD = 1
DIR_REVERSE = 2


class LowLevelClient:
    def __init__(
        self,
        port="/dev/serial0",
        baud=115200,
        tx_hz=20,
        input_timeout=0.3,
    ):
        self.ser = serial.Serial(
            port,
            baud,
            timeout=0,
        )

        self.tx_period = 1.0 / tx_hz
        self.input_timeout = input_timeout

        self.intent = ControlIntent.stop("init")
        self.last_intent_time = 0.0

        self.telemetry = {
            "state": "-",
            "rpm": 0.0,
            "target": 0.0,
            "ramp": 0.0,
            "pwm": 0,
            "dist": 0,
            "hb": 0,
            "raw": "",
            "last_rx": 0.0,
        }

        self.running = False
        self.lock = threading.Lock()

    def start(self):
        self.running = True

        threading.Thread(
            target=self._tx_loop,
            daemon=True,
        ).start()

        threading.Thread(
            target=self._rx_loop,
            daemon=True,
        ).start()

    def close(self):
        self.set_intent(ControlIntent.stop("close"))
        time.sleep(0.2)

        self.running = False

        try:
            self.ser.close()
        except Exception:
            pass

    def set_intent(self, intent: ControlIntent):
        intent.clamp()

        with self.lock:
            self.intent = intent
            self.last_intent_time = time.time()

    def get_telemetry(self):
        with self.lock:
            return dict(self.telemetry)

    def get_intent(self):
        with self.lock:
            return self.intent

    def _tx_loop(self):
        while self.running:
            self._send_current_packet()
            time.sleep(self.tx_period)

    def _rx_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = (
                        self.ser.readline()
                        .decode(errors="ignore")
                        .strip()
                    )

                    if line:
                        self._parse_telemetry(line)
                else:
                    time.sleep(0.005)

            except Exception:
                time.sleep(0.05)

    def _send_current_packet(self):
        with self.lock:
            intent = self.intent
            age = time.time() - self.last_intent_time
            tel = dict(self.telemetry)

        safe_intent = intent

        if age > self.input_timeout:
            safe_intent = ControlIntent.stop("input_timeout")

        if tel["hb"] == 0:
            safe_intent = ControlIntent.stop("nano_hb_lost")

        if tel["state"] not in ("RUN", "RUNNING"):
            safe_intent = ControlIntent.stop("nano_not_running")

        packet = self._intent_to_packet(safe_intent)
        self.ser.write(packet)

    def _intent_to_packet(self, intent: ControlIntent):
        if intent.brake or abs(intent.speed) < 0.01:
            direction = DIR_STOP
            throttle = 0
        elif intent.speed > 0:
            direction = DIR_FORWARD
            throttle = int(abs(intent.speed) * 255)
        else:
            direction = DIR_REVERSE
            throttle = int(abs(intent.speed) * 255)

        throttle = max(0, min(255, throttle))

        servo = int(90 + intent.steer * 40)
        servo = max(50, min(130, servo))

        packet = bytearray(5)
        packet[0] = HEADER
        packet[1] = direction
        packet[2] = throttle
        packet[3] = servo
        packet[4] = packet[0] ^ packet[1] ^ packet[2] ^ packet[3]

        return packet

    def _parse_telemetry(self, line: str):
        items = re.findall(
            r"([A-Za-z_]+)\s*[:=]\s*([\-A-Za-z0-9_.]+)",
            line,
        )

        if not items:
            return

        with self.lock:
            self.telemetry["raw"] = line
            self.telemetry["last_rx"] = time.time()

            for k, v in items:
                k = k.lower()

                try:
                    if k in ("state", "fsm"):
                        self.telemetry["state"] = v
                    elif k in ("rpm", "currentrpm"):
                        self.telemetry["rpm"] = float(v)
                    elif k in ("target", "targetrpm"):
                        self.telemetry["target"] = float(v)
                    elif k in ("ramp", "ramprpm"):
                        self.telemetry["ramp"] = float(v)
                    elif k in ("pwm", "motorpwm"):
                        self.telemetry["pwm"] = int(float(v))
                    elif k in ("dist", "distance", "tf"):
                        self.telemetry["dist"] = int(float(v))
                    elif k in ("hb", "heartbeat"):
                        self.telemetry["hb"] = int(float(v))
                except ValueError:
                    pass
