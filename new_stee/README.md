# Autodrive Ground Control Station

Run with Raspberry Pi OS system Python so the apt-installed `Picamera2` module
is available:

```bash
cd /home/hthnh2/Desktop/autodriving/new_stee
python web_control.py
```

Open `http://<pi-ip>:8000/manual`. The lane-stage dashboard is at
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
FRONT_CAMERA_ID=1 \
DOWN_CAMERA_ID=0 \
python web_control.py
```

The default camera mapping is camera 1 as the physical front-facing camera and
camera 0 as the physical downward lane camera. `FRONT_CAMERA_ID` and
`DOWN_CAMERA_ID` can still override those defaults.

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
- `/lane/debug.mjpg` (down camera rotated 180° with Stage 1 lane overlay)

Camera resolution and stream rate default to 640×480 at 15 FPS. Override with
`CAMERA_WIDTH`, `CAMERA_HEIGHT`, and `CAMERA_FPS`.

Stage 1 uses a preset-driven HSV threshold, ROI, and contour centroid to report
normalized steering. Algorithm and Nano control both start disabled. Vision-only
dry-run mode is available by starting the algorithm without enabling control.
Optional Nano lane control is separately gated by lane validity, confidence,
and manual override; any accepted manual action immediately disables lane
control.

## Lane color presets

The default and recommended indoor test setup is black electrical tape on a
bright tile floor:

```bash
LANE_COLOR_PRESET=black_on_bright python web_control.py
```

White tape on a dark surface is also supported:

```bash
LANE_COLOR_PRESET=white_on_dark python web_control.py
```

Available Stage 1 environment settings and their defaults are:

```text
LANE_COLOR_PRESET=black_on_bright
LANE_ROI_Y_START=280
LANE_ROI_Y_END=480
LANE_MIN_AREA=300
LANE_MORPH_KERNEL=5
LANE_KP=0.003
LANE_STEER_SIGN=1
LANE_BASE_SPEED=0.12
LANE_MIN_CONFIDENCE=0.005
LANE_LOST_TIMEOUT=0.3
LANE_MAX_STEER_NORM=0.6
```

All Stage 1 settings can also be changed live from `/autodrive`; a server
restart is not required. The WebSocket payload reports the active settings,
detection result, and actual commanded speed/steering.

## Safe Stage 1 test order

1. Start Algorithm only, leaving Nano control disabled.
2. Tune preset, ROI, morphology, and minimum area until confidence is stable.
3. Check `steer_sign` while watching dry-run steering.
4. Set `base_speed` to 0.10–0.12.
5. Enable Nano Control in a clear test area with emergency stop accessible.
6. Increase `kp` slowly.

When control is active, a lost or low-confidence lane sends an immediate stop;
`LANE_LOST_TIMEOUT` controls how often that stop is reasserted while loss
persists. Emergency Stop disables both Stage 1 flags and sends an explicit
stop. Stop Algorithm and Disable Nano Control also stop any active lane command.

## Stage selector and Stage 2

The Autodrive page can switch live between two educational detectors on the
same rotated downward-camera feed:

- **Stage 1 — Color Threshold** selects lane-colored pixels, cleans the mask,
  and uses the largest contour centroid.
- **Stage 2 — Canny + Hough** finds intensity edges, groups geometric line
  segments into left/right sides, fits lane lines, and combines center and
  heading error for steering.

Switching stages keeps vision running if it was already started, but always
disables Nano control and sends a stop. This makes side-by-side algorithm
comparison safe and repeatable.

Stage 2 environment defaults are:

```text
LANE_STAGE2_ROI_Y_START=260
LANE_STAGE2_ROI_Y_END=480
LANE_STAGE2_BLUR_KERNEL=5
LANE_STAGE2_CANNY_LOW=50
LANE_STAGE2_CANNY_HIGH=150
LANE_STAGE2_HOUGH_THRESHOLD=30
LANE_STAGE2_MIN_LINE_LENGTH=30
LANE_STAGE2_MAX_LINE_GAP=20
LANE_STAGE2_SLOPE_MIN_ABS=0.35
LANE_STAGE2_SLOPE_MAX_ABS=5.0
LANE_STAGE2_LANE_WIDTH_PX=220
LANE_STAGE2_CENTER_KP=0.003
LANE_STAGE2_HEADING_KP=0.015
LANE_STAGE2_STEER_SIGN=1
LANE_STAGE2_BASE_SPEED=0.10
LANE_STAGE2_MIN_CONFIDENCE=0.15
LANE_STAGE2_LOST_TIMEOUT=0.3
LANE_STAGE2_MAX_STEER_NORM=0.6
```

Every Stage 2 setting shown on `/autodrive` applies live. Canny thresholds
control which intensity transitions become edges. Hough threshold, minimum
line length, and maximum gap control which edge groups become line segments.
Slope limits reject near-horizontal or implausibly steep segments. Lane width
provides a center estimate when only one side is visible. `center_kp` corrects
lateral displacement while `heading_kp` corrects the fitted lane direction.

### Safe Stage 2 test order

1. Select Stage 2.
2. Start Algorithm only.
3. Tune ROI until the road area is visible.
4. Tune Canny thresholds until lane-tape edges are visible.
5. Tune Hough threshold, minimum line length, and maximum line gap until lines
   are stable.
6. Check left/right line counts and confidence.
7. Tune center and heading gains with Nano control disabled.
8. Set base speed low, around 0.08–0.12.
9. Enable Nano Control in a clear test area.
10. Increase speed and gains slowly.
