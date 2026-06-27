# PROJECT REVIEW — AUTODRIVE

> Phạm vi review: toàn bộ source đang có trong repository tại commit `3e9adaa` (2026-06-22). Không có source firmware Arduino/Nano (`.ino/.cpp/.h`) trong repo, vì vậy các chi tiết phía Nano chỉ có thể suy ra từ code Pi và `README.md`. Cần xác minh trên firmware thực trước khi chạy xe.

## 0. Kết luận nhanh và ranh giới source

Repo đang chứa **hai kiến trúc không tương thích**:

| Nhánh | Vai trò | Trạng thái |
|---|---|---|
| `steeringwheel/` | Xe lái bằng servo, camera/lane/follow, Pi gửi binary UART cho Nano | Nhánh mới/current theo các commit gần nhất; chưa hoàn thiện |
| `tank-like/` | Xe vi sai hai bánh, NRF → Redis → lệnh text UART; teach/repeat bằng IMU | Legacy/prototype độc lập |
| `temp.py`, `temp/` | Bản sao/thử nghiệm cũ, gồm UART text và ultrasonic telemetry | Không nằm trong luồng chạy chính |

Tài liệu này ưu tiên kiến trúc `steeringwheel/`. Không được lấy baudrate hoặc packet của `tank-like/` để dùng cho Nano của `steeringwheel/`.

## 1. Tổng quan kiến trúc

### 1.1 Module chính của `steeringwheel/`

| File/folder | Trách nhiệm |
|---|---|
| `steeringwheel/main.py` | Entrypoint desktop PyQt5 |
| `steeringwheel/web_gui/server.py` | Entrypoint FastAPI/web (khởi tạo `ControlHub` ở lúc import) |
| `steeringwheel/manu.py` | Manual keyboard CLI riêng, không đi qua `ControlHub` |
| `steeringwheel/control/hub.py` | Trạng thái trung tâm, chọn mode, loop điều khiển 20 Hz, encode drive command |
| `steeringwheel/control/modes/` | Manual, Follow, Auto L0/L1 |
| `steeringwheel/protocol.py` | Binary framing và XOR CRC |
| `steeringwheel/uart_driver.py` | UART `/dev/serial0`, 115200 baud |
| `steeringwheel/core/camera_manager.py` | Hai camera Picamera2, mỗi camera một capture thread |
| `steeringwheel/algorithms/` | Lane geometry, KCF target tracking, lidar sector processing |
| `steeringwheel/core/rplidar_module.py` | Wrapper RPLidar bằng subprocess SDK; chưa nối vào hub |
| `steeringwheel/core/tfluna_module.py` | Đọc TF-Luna I2C; chưa nối vào hub |
| `steeringwheel/gui/` | Desktop GUI theo ba màn Manual/Follow/Auto |
| `steeringwheel/web_gui/` | Web UI, joystick, status WebSocket, camera MJPEG |

### 1.2 Luồng dữ liệu hiện tại

```text
Desktop buttons / Web joystick / keyboard CLI
                  │
                  ▼
       input_state {speed, steer, mode, level}
                  │
Camera 1 ──► KCF follow ─┐
Camera (intended: 0/down) ─► lane error ├──► Mode.process() ──► clamp ──► UART frame
Lidar ─────► hiện là mock┘       (20 Hz)        speed ±200   /dev/serial0
                                                steer 45..135
                                                        │
                                                        ▼
                                                     Nano
                                                        │
                                             L298N motors + servo
```

- Camera ID `1` được coi là camera trước; ID `0` là camera nhìn xuống và có vẻ được dự định cho lane. Tuy nhiên current hub chưa hề khởi tạo `LaneAlgoGeometry`, nên camera lane chưa được gắn chính thức. Cả hai camera cấu hình `640x480 @ 30 FPS`, RGB888.
- Desktop/web không tự start camera khi hub khởi tạo; người dùng phải bật camera ở UI/API.
- `ControlHub._loop()` chạy thread daemon mỗi `0.05 s` và luôn gửi lệnh mới.
- Lidar thật và TF-Luna hiện **không nằm trong quyết định điều khiển**.

### 1.3 Phân chia Pi 5 và Nano

| Raspberry Pi 5 | Arduino/Nano (suy ra, firmware không có trong repo) |
|---|---|
| GUI/web/keyboard, Picamera2, OpenCV lane/KCF, mode arbitration, clamp, tạo UART packet | Parse UART, đặt chiều/PWM hai DC motor, đặt góc servo |
| RPLidar subprocess và TF-Luna I2C prototype | Theo README: L298N ENA D9, ENB D10; DIR D4-D7; servo D11 |
| Chưa có feedback control tốc độ bánh | README nói giữ lệnh cuối; không thấy watchdog/encoder/telemetry được chứng minh bằng source |

## 2. Giao tiếp Pi ↔ Nano

### 2.1 Giao thức thực sự được code `steeringwheel/` gửi

| Thuộc tính | Giá trị |
|---|---|
| Port | `/dev/serial0` |
| Baudrate | `115200` |
| Timeout đọc | `0.1 s` |
| Chu kỳ drive | `50 ms` (20 Hz) |
| CRC | XOR tất cả byte từ header đến hết payload |

Frame có dạng:

```text
[0xAA, LENGTH, CMD, PAYLOAD..., CRC_XOR]
```

- `LENGTH = 1 + len(PAYLOAD)` (tính `CMD + PAYLOAD`, không tính header/length/CRC).
- `CMD_DRIVE = 0x01`, payload 5 byte, nên drive frame dài 9 byte:

| Byte | Ý nghĩa |
|---:|---|
| 0 | `0xAA` |
| 1 | `0x06` |
| 2 | `0x01` (`CMD_DRIVE`) |
| 3 | `steer`, code clamp `45..135`, center `90` |
| 4 | `dir_l`: `1` forward, `2` backward |
| 5 | `speed_l`: `0..200` trong hub |
| 6 | `dir_r`: `1` forward, `2` backward |
| 7 | `speed_r`: `0..200` trong hub |
| 8 | XOR byte 0..7 |

`speed`/throttle phía API là số có dấu `-200..200`; dấu được đổi thành direction, trị tuyệt đối thành PWM byte. Hai bánh luôn nhận cùng speed/direction trong `steeringwheel/`, nên steering chỉ qua servo.

### 2.2 Ping, heartbeat và safety

- `CMD_PING = 0xF0`; hub gửi đúng **một lần** khi khởi tạo rồi không đọc phản hồi.
- `manu.py` gửi ping và tìm byte `0xAC` trong tối đa 1 giây, nhưng vẫn tiếp tục chạy nếu không thấy ACK.
- Drive frame được gửi đều 20 Hz và có thể xem như heartbeat từ Pi, nhưng không có sequence number, timestamp, ACK, link-state hoặc retry policy.
- Theo README, Nano giữ lệnh cuối nếu mất UART. Nếu firmware thật đúng như vậy, mất process/cáp UART khi xe đang chạy sẽ làm xe **tiếp tục chạy**, không fail-safe.
- Emergency phía Pi là latch boolean: khi true, hub gửi `speed=0`, `steer=90`. Có `clear_emergency()` nhưng desktop/web không cung cấp nút/API clear.

### 2.3 Mâu thuẫn với README và code legacy

`README.md` mô tả protocol cũ **6 byte, 500000 baud**:

```text
[0xAA, speedL, speedR, servo, flags, additive_checksum]
flags bit0=dirL forward, bit1=dirR forward
```

Protocol này không được `steeringwheel/protocol.py` sử dụng. Trong protocol hiện tại **không có byte `flags`**; direction là hai byte riêng (`dir_l`, `dir_r`). `tank-like/` lại dùng `/dev/serial0 @ 9600` và chuỗi text như `9:MOTOR:0:1:150\n`. Ba biến thể này không tương thích.

Ngoài ra checksum của example trong README cũng sai theo chính công thức được ghi: `150+150+90+3 = 393`, modulo 256 phải là `0x89`, không phải `0x1F`.

### 2.4 Telemetry Nano → Pi

- Luồng current `steeringwheel/`: **không có telemetry parser**; hub không đọc UART sau ping.
- `temp/test_uart.py` (prototype, không được import) từng gửi `8:REQ_DATA` mỗi `0.5 s`, parse dòng `Sensor N: D cm` cho 6 cảm biến siêu âm và stop khi bất kỳ `0 < D < 15 cm`.
- `tank-like/control/console_control.py` có đọc text khởi động từ Arduino nhưng không định nghĩa telemetry chuẩn.
- Vì không có firmware Nano, không thể khẳng định Nano hiện có phát telemetry nào.

## 3. Low-level control

| Hạng mục | Hiện trạng |
|---|---|
| Motor driver | README xác nhận L298N, hai DC motor. Không có firmware để kiểm chứng pin/timer hiện tại |
| Motor command | Open-loop PWM byte; `steeringwheel` clamp `0..200`, README nói phần cứng nhận `0..255` |
| Steering servo | Center `90°`; Pi clamp `45..135°`. README nói protocol/hardware cho phép `0..180°` |
| Encoder | Không có code đọc encoder trong toàn repo |
| RPM control | Không có RPM estimate, target RPM, PID tốc độ hoặc odometry |
| Soft start/ramp | Hub/CLI không có ramp trung tâm. Desktop Manual tăng `+5/50 ms`, giảm tự do `10/50 ms`; web tăng `10/100 ms`, decay `10/100 ms` |
| Emergency stop | Chỉ ép command về 0 ở Pi; chưa chứng minh braking điện, relay/cutoff hoặc Nano watchdog |
| Shutdown | `manu.py` gửi stop trong `finally`; `ControlHub` không có `stop()/join()/UART close` hoặc shutdown hook |

Giới hạn hiện có: `steer 45..135`, `speed -200..200`, checksum XOR, target mất trong Follow thì stop. Không có giới hạn gia tốc thống nhất, stale-sensor timeout, watchdog Nano, reverse interlock, current/battery/thermal limit, encoder plausibility hoặc command-age check.

`tank-like/control/ui_control.py` là phương án khác: Pi điều khiển L298N trực tiếp qua GPIO 17/27/22/23 và PWM GPIO 18/19 @ 1 kHz. Không được trộn phương án này với `steeringwheel` qua Nano.

## 4. Perception / Autodrive

### 4.1 Camera pipeline

`CameraManager` tạo `Picamera2(camera_num=id)`, preview RGB888 `640x480 @ 30 FPS`; capture thread ghi `latest_frame` dưới lock. UI đọc frame khoảng 30 ms/lần. Không có camera calibration/undistort trong nhánh `steeringwheel`.

### 4.2 Lane detection và steering

`LaneAlgoGeometry.update()` trên camera down (cam ID do caller truyền):

1. RGB → gray → Gaussian blur `(5,5)` → Canny `(50,150)`.
2. Lấy 30% đáy ảnh (`y >= 0.7h`), cộng histogram theo cột.
3. Peak nửa trái/phải là `left_x/right_x`; center lane là trung bình.
4. Nếu lane width `<50 px`, confidence = 0 và giữ error cũ.
5. `error = clamp((lane_center-car_center)/(lane_width/2), -1, 1)`.
6. Auto L1: `steer = 90 + error * kp_steering * 100`.

Tuning hiện tại: `kp_steering=0.004`, nên toàn biên error chỉ tạo ±`0.4°`; gần như không lái. Confidence/timestamp không được kiểm tra trước khi dùng. Hub còn thiếu `select_lane_algo()`, `start_lane_algo()`, `stop_lane_algo()`, vì vậy UI hiện không thể thực sự bật lane algorithm.

### 4.3 Mode Follow

- Người dùng kéo bbox trên camera trước; `TargetTracker` dùng OpenCV KCF.
- Error ngang: `(target_center_x-frame_center_x)/frame_center_x`, xấp xỉ `[-1,1]`.
- Nếu thấy target: `steer = 90 + error*45`, speed cố định `100`; target area không được dùng để giữ khoảng cách.
- Nếu mất target: center và stop.
- `runtime_params.kp_steering` được đọc trong `follow_arbitration()` nhưng kết quả không dùng; steering vẫn hard-code `*45`.
- Nếu tracker đã init nhưng frame `None`, `cv2.cvtColor(None, ...)` có thể làm chết control thread.

### 4.4 Lidar và TF-Luna

- `LidarProcessor`: front ±30°, left 30..90°, right -90..-30°; `stop_threshold=0.5` (ngầm hiểu mét); chọn phía có khoảng trống lớn hơn.
- Follow hiện truyền **mock scan toàn bộ 2.0 m**, không đọc RPLidar thật.
- `follow_arbitration()` nhận `lidar_result` nhưng hoàn toàn bỏ qua; vật cản không làm giảm/stop speed.
- `RPLidar360` chạy SDK binary hard-code `/home/hthnh2/rplidar_sdk/.../ultra_simple`, port `/dev/ttyUSB0`, baud `460800`. Module chưa được khởi tạo trong hub; cần xác minh đơn vị SDK (thường output có thể không phải mét) trước khi đưa vào threshold `0.5`.
- `LidarManager` hiện lỗi tên/import (`RPLidarModule`, `get_front_obstacles` không tồn tại).
- `TFLuna` đọc I2C bus 1, address `0x10`, trả distance raw (thường cm theo thiết bị), nhưng chưa được dùng ở mode nào. `core/te.py` chỉ là script test chạy vòng lặp ngay khi import.

### 4.5 Tuning đang tồn tại

| Tham số | Giá trị | Nơi dùng |
|---|---:|---|
| Control loop | 20 Hz | `hub.py`, `manu.py` |
| Camera | 640x480, 30 FPS | `hub.py` |
| `MAX_SPEED` | 200 | hub/CLI/UI |
| Servo clamp | 45..135°, center 90° | hub/manual/web |
| Lane Canny | 50/150 | lane geometry |
| Lane ROI | 30% đáy ảnh | lane geometry |
| Min lane width | 50 px | lane geometry |
| `kp_steering` | 0.004 | Auto L1 (Follow đọc nhưng không dùng) |
| Lidar sectors | front 60°, side 60° | `LidarProcessor` |
| Stop threshold | 0.5 | processor/UI mock; chưa tác động actuator |
| Follow speed | 100 fixed | `behavior.py` |

## 5. UI / Manual control

### 5.1 Entrypoint

- Desktop: chạy từ thư mục `steeringwheel/` bằng `python main.py`.
- Web: cần chạy FastAPI/uvicorn với working directory phù hợp vì static/template path là relative; module chính `web_gui/server.py`.
- CLI keyboard: `python manu.py`.

### 5.2 Mode

| Mode | Điều khiển steer | Điều khiển speed | Ghi chú |
|---|---|---|---|
| Manual | UI/web/CLI | UI/web/CLI | Hoạt động nhất trong các mode |
| Auto L0 | Manual | Manual | Tương đương Manual |
| Auto L1 | Lane error nếu algo running | User input | Lane lifecycle chưa nối nên thực tế thường fallback manual steer |
| Auto L2 | Không có implementation | Không có implementation | `set_level(2)` giữ lại object mode trước đó |
| Follow | KCF target | Cố định 100 hoặc 0 khi mất target | Không tránh vật cản thật |

### 5.3 Tác động của control UI

| Control | Biến/tác động |
|---|---|
| Desktop Forward/Backward hold | `ManualScreen.cur_speed`, bước ±5 mỗi 50 ms; thả thì về 0 bước 10 |
| Desktop Left/Right hold | `cur_steer`, bước ±10 mỗi 100 ms; không tự center |
| Desktop Center | `cur_steer=90` |
| Desktop Emergency | local speed 0/steer 90 rồi `hub.emergency=True` |
| Web joystick dọc | `currentSpeed`, bước ±10 mỗi 100 ms, clamp ±200, tự decay |
| Web joystick ngang | `currentSteer`, bước ±5, clamp 45..135, không tự center |
| Web CONTROL OFF | Chỉ dừng gửi input mới; **không gửi stop**, hub giữ input cuối |
| Mode buttons | `hub.set_mode()`; desktop nút AUTO chỉ đổi màn hình, mode chỉ đổi khi bấm level |
| Auto speed +/- module | Chỉ đổi label/local `target_speed`; không gọi hub |
| Auto lane ENABLE | Chỉ đổi label/local flag; không bật algorithm |
| Camera buttons/API | Start/stop cam 0/1 |
| Lidar buttons | Chỉ toggle mock/visual state |

`manu.py`: `W/S` đổi speed ±10; `A/D` đổi steer ±5; Space đặt speed 0; `Q` thoát và gửi stop.

## 6. Vấn đề kỹ thuật và rủi ro an toàn

### 6.1 Ưu tiên cao / có thể gây mất an toàn

1. **Không có Nano watchdog** được chứng minh; README nói giữ command cuối khi mất link.
2. **Ba protocol/baudrate mâu thuẫn** trong cùng repo: binary 115200 current, binary 500000 README, text 9600 legacy.
3. Emergency chỉ là software state trên Pi; loop crash hoặc Pi mất điện không bảo đảm stop. Không có ACK/telemetry xác nhận motor đã dừng.
4. `ControlHub._loop()` không bắt exception; lỗi camera/tracker/UART có thể làm thread chết mà Nano giữ throttle cuối.
5. Follow lidar là mock và arbitration bỏ qua obstacle result; UI dễ tạo cảm giác lidar đang bảo vệ trong khi thực tế không có.
6. Web `CONTROL OFF`, chuyển mode, đóng browser hoặc mất WebSocket không gửi zero command.
7. Sensor freshness không được kiểm tra: lane có thể dùng error cũ; follow không có timestamp; RPLidar không có scan-age.
8. Không có shutdown lifecycle cho hub/camera/UART; không gửi stop khi GUI/web process kết thúc bình thường hoặc lỗi.

### 6.2 Code rối/trùng/chưa nối

- `ControlHub.start_camera/stop_camera` bị định nghĩa lặp.
- GUI cũ theo mixin còn gọi API/field không tồn tại: `init_follow_tracker`, lane lifecycle, `start_follow/stop_follow`, `manual_speed`, `active_source`, `get_mock_scan`.
- `lane_extend_window.py` có syntax error do quote lồng trong f-string, đồng thời thiếu import `QTimer/QImage/QPixmap/Qt` và kiểm tra `hasattr` sai kiểu (`lane` là dict).
- `LidarManager` dùng symbol không tồn tại.
- Auto module speed/steering/avoid chủ yếu là placeholder, không nối control path.
- `set_mode("auto")` không tự chọn mode; `set_level(2)` không báo lỗi ở hub và giữ controller cũ.
- `set_input(speed, steer)` không clamp/validate tại boundary và không lock; mode/state đổi giữa GUI/FastAPI thread và control thread không đồng bộ.
- Camera frame trả reference trực tiếp; không copy. Camera stop `join()` không timeout.
- `README.md` đã stale so với protocol code.
- `tank-like/` chứa logic điều khiển thứ hai (Redis, NRF, UART text, direct GPIO) và `temp*` chứa bản sao, làm khó xác định source of truth.
- `tank-like/T_R_navigation_method/repeat.py` có PID yaw prototype nhưng gọi `set_target()` hai lần liên tiếp: absolute-yaw result bị ghi đè bởi yaw-rate PID; không nên tái sử dụng nguyên trạng.

### 6.3 Global/state quan trọng

| State | Ý nghĩa |
|---|---|
| `ControlHub.input_state` | Command/mode/level do UI đặt |
| `ControlHub.runtime_params` | `stop_threshold`, `kp_steering` |
| `ControlHub.emergency` | E-stop latch trên Pi |
| `last_speed`, `last_steer` | Command đã tính gần nhất, không phải telemetry thực |
| `lane_state` | error/confidence/timestamp/debug |
| `follow_state` | target visibility/error/area và output debug |
| `mode` | Object controller đang active; có thể lệch với `input_state["mode"]` ở Auto |

### 6.4 Điểm sửa phù hợp khi thiết kế thuật toán tiếp theo

| Muốn thêm | Điểm tích hợp nên dùng |
|---|---|
| PID steering | Thay công thức trong `Level1Mode.process()`, nhưng trước hết chuẩn hóa input error/timestamp và output steer |
| Pure pursuit | Tạo mode/controller mới nhận path + pose, trả curvature/steer; không nhét vào lane image processor |
| Speed planner | Tạo lớp planner giữa perception/state và actuator command; thay speed fixed trong `follow_arbitration()`/user speed L1 |
| Wheel-speed PID | Cần encoder telemetry + low-level Nano API; vòng nhanh nên ở Nano, Pi gửi target speed |
| Obstacle safety | Tạo safety supervisor sau mode output và trước `_send_drive()`, dùng dữ liệu sensor có timestamp/health |
| Ramp/jerk limit | Một limiter thống nhất ngay trước actuator mapping, không để mỗi UI tự ramp |

## 7. Đề xuất API điều khiển sạch

Tách “ý định chuyển động”, “trạng thái safety” và “lệnh actuator”. Interface tối thiểu:

```python
from dataclasses import dataclass
from enum import Enum

class ControlMode(str, Enum):
    MANUAL = "manual"
    FOLLOW = "follow"
    LANE_KEEP = "lane_keep"
    PATH_FOLLOW = "path_follow"

class SafetyState(str, Enum):
    OK = "ok"
    DEGRADED = "degraded"
    STOPPING = "stopping"
    EMERGENCY = "emergency"

@dataclass(frozen=True)
class ControlIntent:
    desired_speed: float       # m/s, signed; không dùng PWM làm đơn vị public
    desired_steer: float       # rad hoặc deg, chọn một và ghi rõ; center = 0
    brake: float               # 0.0..1.0
    safety_state: SafetyState
    control_mode: ControlMode
    timestamp: float           # monotonic time
    source: str                # ui, follow, lane, planner...
```

Pipeline khuyến nghị:

```text
Input/Perception → Mode/Planner → ControlIntent
              → SafetySupervisor (freshness, obstacle, e-stop)
              → RateLimiter
              → VehicleMapper (m/s→PWM, steer zero→servo angle)
              → SerialTransport (seq/CRC/ACK)
```

Quy ước nên chốt:

- `desired_speed` là tốc độ vật lý; PID/mapper mới chuyển sang PWM.
- `desired_steer=0` là thẳng; dấu trái/phải ghi rõ, tránh để mọi thuật toán biết servo center 90°.
- `brake>0` ưu tiên hơn throttle; `EMERGENCY` luôn tạo brake/zero throttle bất kể mode.
- Mỗi intent/sensor có monotonic timestamp và timeout. Intent quá hạn phải stop.
- Chỉ một `SafetySupervisor` có quyền phát lệnh cuối; UI/mode không gọi serial trực tiếp.
- Protocol kế tiếp nên có version, sequence, command age/heartbeat, Nano watchdog và telemetry ACK + encoder RPM + battery/fault bits.

## 8. Source legacy tham khảo (không thuộc control path current)

- `tank-like/nrf_receiver.py`: NRF24 payload 11 bool + 4 int16 → Redis `nrf:raw_data`.
- `tank-like/control/control_process.py`: joystick vi sai → speed levels `[0,60,100,150,200,255]` → Redis `vehicle:commands_processed`.
- `tank-like/uart_sender.py`: Redis → text UART `/dev/serial0 @ 9600`, chỉ gửi field thay đổi.
- `tank-like/T_R_navigation_method/teach.py`: log control + camera + IMU/IPS.
- `tank-like/T_R_navigation_method/repeat.py`: replay log, Madgwick và PID yaw prototype (`Kp=1.5, Ki=0.2, Kd=0.3`, output ±127).
- `tank-like/calibrate_imu/` và `ips/`: calibration IMU/camera, AprilTag experiments.
- `temp/test_uart.py`: prototype ultrasonic reverse telemetry; không phải contract hiện hành.

## 9. Việc cần xác minh trước lần thiết kế/ chạy xe tiếp theo

1. Lấy đúng firmware đang flash trên Nano và đưa vào repo.
2. Đo/xác nhận baudrate và frame thật bằng serial test/logic analyzer; chọn một protocol duy nhất.
3. Xác nhận servo cơ khí: center, đảo chiều, giới hạn không chạm cơ cấu.
4. Xác nhận PWM tối thiểu làm xe chuyển động, deadband, chiều motor và hành vi brake/coast.
5. Thêm/kiểm tra Nano watchdog trước mọi thử nghiệm autonomous có bánh chạm đất.
6. Xác định đơn vị/khung tọa độ của RPLidar và TF-Luna, cùng timestamp/failure behavior.
7. Nếu cần điều khiển tốc độ thật: bổ sung encoder, pulses/rev, wheel diameter, gear ratio và telemetry RPM.
