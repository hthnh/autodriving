# RC Car Control Protocol – Arduino Nano

## Overview
This firmware is optimized for **high-speed real-time control** of an RC car using:
- 2 DC motors (L298N)
- 1 servo steering motor
- Communication via **UART binary protocol** from Raspberry Pi 5

The system executes every new command immediately upon receiving it, no delay.

---

## ⚙️ Hardware Pinout
| Function       | Pin | Note |
|----------------|-----|------|
| L298N ENA      | D9  | PWM (Timer1A) |
| L298N IN1      | D5  | DIR Left |
| L298N IN2      | D4  | DIR Left |
| L298N IN3      | D6  | DIR Right |
| L298N IN4      | D7  | DIR Right |
| L298N ENB      | D10 | PWM (Timer1B) |
| Servo          | D11 | PWM (Timer2) |
| UART RX/TX     | D0/D1 | 500000 baud |

---

## 🧩 Protocol Format
Each packet is **6 bytes** total:

| Byte | Name | Description |
|------|------|-------------|
| 0 | `HEADER` | Always `0xAA` |
| 1 | `speedL` | Left motor speed (0–255) |
| 2 | `speedR` | Right motor speed (0–255) |
| 3 | `servo`  | Servo angle (0–180) |
| 4 | `flags`  | Bit0: dirL (1 = forward), Bit1: dirR (1 = forward) |
| 5 | `checksum` | = (speedL + speedR + servo + flags) & 0xFF |

Example (Forward 150,150 – Servo 90°):
0xAA 0x96 0x96 0x5A 0x03 0x1F


---

## 🧠 Behavior
- Every valid packet instantly updates motor PWM + direction + servo angle.
- Invalid checksum → packet ignored.
- The Nano holds the **last command** until a new valid one arrives.
- UART speed = **500,000 bps** → suitable for 50–100Hz control rate from Raspberry Pi 5.

---

## 🛠️ Optional Extensions
You can easily extend this protocol to include:
- IMU feedback (GY-87)
- Encoder data (wheel speed)
- Battery voltage
by adding more bytes or defining separate packet headers.

---

## 🧪 Recommended Serial Test (Python)
```python
import serial, struct

ser = serial.Serial('/dev/ttyUSB0', 500000)
def send(speedL, speedR, servo, dirL=True, dirR=True):
    flags = (dirL << 0) | (dirR << 1)
    chksum = (speedL + speedR + servo + flags) & 0xFF
    packet = struct.pack('BBBBBB', 0xAA, speedL, speedR, servo, flags, chksum)
    ser.write(packet)

send(150, 150, 90)
```

🧭 Notes

Do not use delay() in the loop — fully event-driven.

Servo frequency = 50Hz (standard analog servo).

Works perfectly with Raspberry Pi 5 UART at 3.3V logic.
