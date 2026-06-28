# Autodrive Ground Control Station

Run with Raspberry Pi OS system Python so the apt-installed `Picamera2` module
is available:

```bash
cd /home/hthnh2/Desktop/autodriving/new_stee
python web_control.py
```

Open `http://<pi-ip>:8000/manual`. The future autonomous-mode shell is at
`http://<pi-ip>:8000/autodrive`.

## Dependencies

Minimum Python packages:

```bash
python -m pip install fastapi "uvicorn[standard]" pyserial
```

Raspberry Pi OS normally supplies camera support through apt:

```bash
sudo apt install python3-picamera2 python3-opencv
```

Optional Python lidar driver:

```bash
python -m pip install --user --break-system-packages rplidar-roboticia
```

RPLidar model 65 (C1) uses Slamtec's `ultra_simple` SDK because the Python
package does not implement its ultra scan format. Override the binary location
with `RPLIDAR_SDK_BINARY` if needed.

## Hardware configuration

The server detects the common CP210x RPLidar + CH340 Nano pairing. Explicit
environment variables always take precedence:

```bash
NANO_PORT=/dev/ttyUSB1 \
LIDAR_PORT=/dev/ttyUSB0 \
LIDAR_BAUD=460800 \
FRONT_CAMERA_ID=0 \
DOWN_CAMERA_ID=1 \
python web_control.py
```

Force mock lidar:

```bash
USE_MOCK_LIDAR=true python web_control.py
```

Require real lidar and expose an error instead of falling back:

```bash
USE_MOCK_LIDAR=false LIDAR_PORT=/dev/ttyUSB0 LIDAR_BAUD=460800 python web_control.py
```

Camera stream endpoints:

- `/camera/front.mjpg`
- `/camera/down.mjpg`

Camera resolution and stream rate default to 640×480 at 15 FPS. Override with
`CAMERA_WIDTH`, `CAMERA_HEIGHT`, and `CAMERA_FPS`.
