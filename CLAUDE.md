# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Ground Control Station (GCS) + onboard software for **ROV MAIVS EVO**, built for the KKI 2026 (Kontes Kapal Indonesia) competition. Code, comments, and UI strings are predominantly in **Bahasa Indonesia** — match that when editing existing files.

The system spans three physical machines/boards connected over Ethernet/WiFi and USB serial:

1. **Teensy 4.1** (`main.cpp`) — low-level firmware: reads MPU6050 (IMU), HMC5883 (compass), MS5837 (depth), drives 6 thrusters + 2 servos (tilt arm, gripper), runs depth-hold and IMU-stabilization PID loops, arming/safety logic. Talks to the Jetson over USB serial.
2. **Jetson** (onboard companion computer, IP `192.168.1.10`) — runs `bridge_jetson.py` (serial↔UDP bridge to the Teensy, with watchdog failsafe) and `start_camera.sh` (GStreamer pipelines streaming two cameras via RTP/H264 to the GCS).
3. **GCS laptop** (IP `192.168.1.5`) — runs the ROS 2 package `src/rov_kki26`, launched via `start_gcs.sh`.

## Build & run

ROS 2 distro: **Jazzy**. Package build type: `ament_python`.

```bash
# Build (from workspace root)
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rov_kki26
source install/setup.bash

# Run the full GCS stack (opens one gnome-terminal per node)
./start_gcs.sh              # prompts to enable recording
./start_gcs.sh --record     # force recording on
./start_gcs.sh --no-record  # force recording off

# Run a single node directly
ros2 run rov_kki26 video_receiver --ros-args -p record:=true
ros2 run rov_kki26 telemetry_receiver
ros2 run rov_kki26 rov_dashboard
ros2 run rov_kki26 qr_scanner
ros2 run rov_kki26 mavis_gamepad
ros2 run rov_kki26 data_logger

# Jetson side (run on the Jetson, not the GCS laptop)
python3 bridge_jetson.py
./start_camera.sh
```

Tests (standard `ament_python` linters, run via colcon or pytest directly on `src/rov_kki26`):

```bash
colcon test --packages-select rov_kki26
colcon test-result --verbose

# Or a single check:
pytest src/rov_kki26/test/test_flake8.py
pytest src/rov_kki26/test/test_pep257.py
pytest src/rov_kki26/test/test_copyright.py
```

`build/`, `install/`, and `log/` are colcon-generated output and currently present in the working tree — don't hand-edit files under them; regenerate with `colcon build`.

## Architecture

### Network protocol (GCS ↔ Jetson ↔ Teensy)

- **Command path** (GCS → Jetson, UDP port `5006`): `mavis_gamepad_node.py` reads a gamepad (pygame) or falls back to keyboard (pynput), and sends a 10-field CSV string at 20 Hz:
  `surge,yaw,roll,tilt,heave,tiltArm,gripper,mode,depth_hold,relevel`.
  It also sends the sentinel strings `GUI_EMERGENCY_ON` / `GUI_EMERGENCY_OFF` for a hard GUI-triggered kill switch.
- `bridge_jetson.py` (on the Jetson) receives that UDP command, re-frames it as `M:s,y,r,t,heave,tiltArm,gripper,mode,depth_hold,relevel\n` and writes it to the Teensy over serial (`/dev/ttyACM1`, falls back to `/dev/ttyACM0`). It enforces a **0.5 s watchdog**: if no UDP command arrives in that window, it forces mode 3 (emergency/neutral) regardless of what the GUI last sent.
- **Modes**: `1` = manual/remote, `2` = depth-lock, `3` = emergency/kill switch (all thrusters forced to neutral PWM at both the Jetson and Teensy layers — a deliberate double lock).
- **Telemetry path** (Jetson → GCS, UDP port `5005`): `bridge_jetson.py` reads Teensy serial lines prefixed `P:`, `PWM`, or `CMD` and forwards them verbatim via UDP, plus its own `MODE:<status>` heartbeat every 0.5 s. `telemetry_receiver_node.py` (GCS) parses these and republishes onto ROS 2 topics.
- **Video path**: `start_camera.sh` (Jetson) encodes both cameras (H264, `nvv4l2h264enc`) and sends RTP over UDP to ports `5000`/`5001`. `video_receiver_node.py` (GCS) decodes via OpenCV+GStreamer and republishes as ROS 2 `Image` topics.

### ROS 2 topics (all under `/rov/`)

| Topic | Type | Published by | QoS |
|---|---|---|---|
| `/rov/cam1/image_raw`, `/rov/cam2/image_raw` | `sensor_msgs/Image` | `video_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/imu_euler` | `geometry_msgs/Vector3` | `telemetry_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/depth` | `std_msgs/Float32` | `telemetry_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/thruster_pwm` | `std_msgs/Int32MultiArray` | `telemetry_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/servo_data` | `std_msgs/Int32MultiArray` | `telemetry_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/system_mode` | `std_msgs/String` | `telemetry_receiver_node` | RELIABLE depth=10 |
| `/rov/qr_data` | `std_msgs/String` | `qr_scanner_node` | — |

`rov_dashboard_node` (PyQt5 + VTK/pyvistaqt 3D view) and `data_logger_node` subscribe to all of the above; `qr_scanner_node` subscribes to both camera topics.

**QoS consistency is load-bearing, not cosmetic**: every sensor/video publisher-subscriber pair must use identical `BEST_EFFORT`/depth=1 QoS. A mismatch (e.g. a subscriber left on default `RELIABLE`) causes DDS to silently drop delivery — no error, data just never arrives. `/rov/system_mode` is intentionally the one `RELIABLE` topic (low rate, must not be lost). If you add a new sensor topic or subscriber, copy the existing `SENSOR_QOS`/`RELIABLE_QOS` `QoSProfile` objects rather than using default `create_subscription`/`create_publisher` queue-size shorthand.

### Video recording lives inside `video_receiver_node.py` — do not split it out

Recording is deliberately implemented inside `video_receiver_node.py` rather than as a separate subscriber node. A prior attempt at a standalone recorder node subscribing to `/rov/cam1|2/image_raw` caused the live video feed to disappear entirely: each 864×486 BGR frame is ~1.26 MB, and DDS sends a full independent copy over UDP loopback to every subscribing process, so a third subscriber pushed traffic past what the kernel socket buffers (~208 KB) and default MTU fragmentation could sustain — one dropped UDP fragment discards the whole BEST_EFFORT frame. See the module docstring in `video_receiver_node.py` for the full writeup. **Rule: never add a new process that subscribes to `image_raw`** — if you need a new image consumer, add it inside `video_receiver_node.py` or move it to a compressed topic.

Recordings are MJPG-in-AVI specifically because it's crash-safe (each frame stands alone; a process dying mid-write leaves a playable file), unlike MP4/H.264 which needs finalization. Output: `recordings/rov_cam{1,2}_<YYYYMMDD_HHMMSS>.avi`.

### Threading pattern used throughout the GCS nodes

`telemetry_receiver_node.py`, `video_receiver_node.py`, and `rov_dashboard_node.py` all follow the same rule: **never call `publisher.publish()` from a non-executor thread.** UDP/serial/GStreamer I/O happens on a background thread that only writes into a thread-safe buffer (`queue.Queue` or a locked "latest frame" slot); an actual ROS 2 `Timer` callback (which runs on the executor thread) drains the buffer and calls `publish()`. Follow this pattern for any new I/O-driven node — publishing directly from a raw `threading.Thread` causes intermittent DDS deadlocks that look like the feed "freezing then resuming."

### `data_logger_node.py`

Runs as its own node (not embedded in the dashboard) specifically so logging survives a GUI crash and has a lifecycle independent of Qt's event loop. Writes `logs/rov_log_<timestamp>.csv` (flushed per line, so it survives a hard crash) and a periodic/final `logs/rov_log_<timestamp>.json` snapshot. `surge_est` is derived locally from `TKIRI`/`TKANAN` PWM using the same formula as the dashboard's trajectory map — keep the two in sync if that formula changes.

### Non-ROS helper scripts (not part of the `rov_kki26` package)

- `MAVIS GAMEPAD/` — standalone gamepad test scripts (`cek_tombol.py`, `mavis_gamepad.py`), not wired into the ROS package or `setup.py` entry points.
- `Train Trejectory/` — `dataset_logger.py` / `train_velocity_model.py`, offline velocity-model training tooling, unrelated to the live GCS/ROS runtime.
- `test_qr.py` — standalone QR-decoding test script at the workspace root.
