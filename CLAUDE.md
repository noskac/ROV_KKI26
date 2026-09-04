# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Ground Control Station (GCS) + onboard software for **ROV MAIVS EVO**, built for the KKI 2026 (Kontes Kapal Indonesia) competition. Code, comments, and UI strings are predominantly in **Bahasa Indonesia** — match that when editing existing files.

The system spans three physical machines/boards connected over Ethernet/WiFi and USB serial:

1. **Teensy 4.1** (`main.cpp`) — low-level firmware: reads MPU6050 (IMU), HMC5883 (compass), MS5837 (depth), drives 6 thrusters + 2 servos (tilt arm, gripper), runs depth-hold and IMU-stabilization PID loops, arming/safety logic. Talks to the Jetson over USB serial. Developed as a **PlatformIO project that lives on the Jetson**, not in this repo — see "Teensy firmware (PlatformIO, on the Jetson)" under Architecture. This repo's root-level `main.cpp` is a manually-kept-in-sync mirror of that project's `src/main.cpp`.
2. **Jetson** (onboard companion computer, IP `192.168.1.10`, SSH alias `Jetson_KKI26`, user `jetson`) — project dir `~/KKI26/` holds `bridge_jetson.py` (serial↔UDP bridge to the Teensy, with watchdog failsafe) and `start_cameras.sh` (GStreamer pipelines streaming two cameras via RTP/H264 to the GCS). Both run as systemd services rather than launched manually — see "Jetson-side services (systemd)" under Architecture.
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

# Jetson side — bridge & cameras run as systemd services, not launched manually
# Deploy a bridge_jetson.py change: no automated sync — scp it over, then restart the service
scp bridge_jetson.py Jetson_KKI26:/home/jetson/KKI26/bridge_jetson.py
ssh Jetson_KKI26 sudo systemctl restart rov-bridge      # bridge only
ssh Jetson_KKI26 sudo systemctl restart rov-camera      # cameras only
ssh Jetson_KKI26 sudo systemctl start\|stop kki26.target # both together
ssh Jetson_KKI26 journalctl -u rov-bridge -f -o cat      # live bridge log

# Teensy firmware — PlatformIO project lives on the Jetson, not in this repo
ssh Jetson_KKI26
cd ~/Documents/PlatformIO/Projects/KKI26_MCU
~/.platformio/penv/bin/pio run              # build
~/.platformio/penv/bin/pio run -t upload    # build + flash (auto-reboots the Teensy)
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
- **`mode` field** (index 7 of the 10-field command): `1` = manual/remote (default — full operator control), `2` = reserved/unimplemented (accepted by the Teensy's arming check alongside mode 1, but has no distinct behavior anywhere in `main.cpp`; do not treat it as a real mode), `3` = emergency/kill switch (all thrusters forced to neutral PWM at both the Jetson and Teensy layers — a deliberate double lock).
- **`depth_hold` field is a separate boolean, not a mode** (index 8 of the 10-field command — do not confuse with `mode`). It's toggled by gamepad button index 0 (named "B0" in the code's own comments/variables — see `mavis_gamepad_node.py`'s `depthhold_btn_pressed`/`depth_hold_wanted`) or the spacebar on keyboard; each press flips it on/off (toggle, not hold-to-activate). On the rising edge of `depth_hold` turning on, the Teensy locks the setpoint to the **live MS5837 depth reading at that instant** (`depthSetpoint = depth` in `main.cpp`), then a PID loop (`computeDepthPID`) substitutes the heave axis with a correction to hold that depth — surge/yaw/roll/tilt/tiltArm/gripper stay fully under manual control the whole time. Depth-hold is force-disabled whenever `mode == 3` (`depthHoldON = autoCmd.depthHold && autoCmd.mode != 3`), but is otherwise completely independent of the `mode` field.
  - `mavis_gamepad_node.py` builds the *sent* `depth_hold` value from two separate pieces of state, entirely on the GCS side — `main.cpp` is unaware of any of this and needs no changes: `depth_hold_wanted` (the toggle state from B0/spacebar) is the operator's persistent intent, while the actual value transmitted each frame also depends on the heave trigger. Moving the heave trigger away from neutral immediately forces the sent `depth_hold` to `0` (full manual heave), without touching `depth_hold_wanted`. Once heave (`h_val`, the *post-smoothing* PWM — after LPF/slew/snap-to-neutral, not the raw axis) has been back at 1500 continuously for `DEPTH_HOLD_RELOCK_DELAY` (a tunable constant in the node's TUNING block, default 0.5 s, added so ROV momentum settles before the Teensy re-locks its setpoint), the node resumes sending `depth_hold=1` if `depth_hold_wanted` is still `True` — the Teensy sees this as a fresh rising edge and re-locks to whatever depth the ROV is at by then. Pressing B0 while depth-hold is on clears `depth_hold_wanted` and turns it off outright. Entering `mode == 3` also clears `depth_hold_wanted` and the relock timer, so leaving emergency mode never silently re-engages depth-hold.
- **Telemetry path** (Jetson → GCS, UDP port `5005`): `bridge_jetson.py` reads Teensy serial lines prefixed `P:`, `PWM`, or `CMD` and forwards them verbatim via UDP, plus its own `MODE:<status>` heartbeat every 0.5 s. `telemetry_receiver_node.py` (GCS) parses these and republishes onto ROS 2 topics.
- **Video path**: `start_cameras.sh` (Jetson) encodes both cameras (H264, `nvv4l2h264enc`) and sends RTP over UDP to ports `5000`/`5001`. `video_receiver_node.py` (GCS) decodes via OpenCV+GStreamer and republishes as ROS 2 `Image` topics.

### Teensy firmware (PlatformIO, on the Jetson)

The firmware source of truth is a PlatformIO project at `~/Documents/PlatformIO/Projects/KKI26_MCU/` **on the Jetson**, not in this repo. `platformio.ini`: env `teensy41`, `platform = teensy@4.17.0`, `framework = arduino`; libs are Adafruit MPU6050, Adafruit Unified Sensor, Adafruit SSD1306, Adafruit GFX, BlueRobotics MS5837, and Adafruit HMC5883 Unified. The upload step is hijacked to a custom command (`~/teensy_loader_cli/teensy_loader_cli -mmcu=TEENSY41 -w -s -v`, also on the Jetson) so `pio run -t upload` auto-reboots the Teensy instead of requiring a physical button press.

This repo's root-level `main.cpp` is a plain mirror of that project's `src/main.cpp` with no automated sync — edit the PlatformIO copy on the Jetson and copy changes back here, not the other way around, and `diff` the two before trusting either as current.

### Jetson-side services (systemd)

`bridge_jetson.py` and `start_cameras.sh` run as systemd services on the Jetson (`~/KKI26/RUNBOOK_systemd_jetson.md` on the Jetson has the full migration writeup and rationale):

- `rov-bridge.service` runs `bridge_jetson.py`, `rov-camera.service` runs `start_cameras.sh`; both are `PartOf=kki26.target` with `Restart=always`.
- `kki26.target` (`WantedBy=multi-user.target`) starts/stops both together.
- An older mechanism (`kki26.service` running `~/KKI26/autostart.sh`, which backgrounds both processes with `&` then a trailing `wait`) is kept only as a documented rollback. **Never run it enabled alongside the new services**: `autostart.sh`'s `wait` only tracks the wrapper script, so systemd can't detect a child process dying, and both mechanisms running together fight over the same serial device (`/dev/ttyACM0`/`ttyACM1`) and UDP/video ports.
- Prefer `ssh Jetson_KKI26 systemctl ...` / `journalctl` for routine restarts and log-watching over opening a full VS Code Remote-SSH session.
- Judge liveness from the data path (the `MODE:` UDP heartbeat, incoming video frames), not `systemctl is-active` — a service can report active while its process can't actually reach the serial port or a camera.

### ROS 2 topics (all under `/rov/`)

| Topic | Type | Published by | QoS |
|---|---|---|---|
| `/rov/cam1/image_raw`, `/rov/cam2/image_raw` | `sensor_msgs/Image` | `video_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/imu_euler` | `geometry_msgs/Vector3` | `telemetry_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/depth` | `std_msgs/Float32` | `telemetry_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/depth_setpoint` | `std_msgs/Float32` | `telemetry_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/thruster_pwm` | `std_msgs/Int32MultiArray` | `telemetry_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/servo_data` | `std_msgs/Int32MultiArray` | `telemetry_receiver_node` | BEST_EFFORT depth=1 |
| `/rov/system_mode` | `std_msgs/String` | `telemetry_receiver_node` | RELIABLE depth=10 |
| `/rov/depth_hold_status` | `std_msgs/String` | `mavis_gamepad_node` | RELIABLE depth=10 |
| `/rov/qr_data` | `std_msgs/String` | `qr_scanner_node` | — |

`rov_dashboard_node` (PyQt5 + VTK/pyvistaqt 3D view) and `data_logger_node` subscribe to all of the above; `qr_scanner_node` subscribes to both camera topics.

**QoS consistency is load-bearing, not cosmetic**: every sensor/video publisher-subscriber pair must use identical `BEST_EFFORT`/depth=1 QoS. A mismatch (e.g. a subscriber left on default `RELIABLE`) causes DDS to silently drop delivery — no error, data just never arrives. `/rov/system_mode` and `/rov/depth_hold_status` are intentionally `RELIABLE` (low rate, must not be lost — a dropped depth-hold transition would leave the dashboard indicator stuck wrong). If you add a new sensor topic or subscriber, copy the existing `SENSOR_QOS`/`RELIABLE_QOS` `QoSProfile` objects rather than using default `create_subscription`/`create_publisher` queue-size shorthand.

`/rov/depth_hold_status` (values `"OFF"` / `"HOLDING"` / `"MANUAL HEAVE"`) is published by `mavis_gamepad_node.py`, not by the Teensy/telemetry path, and this is deliberate: the Teensy only ever sees the raw, already-toggled `depth_hold` bit sent over the 10-field CSV command, so it cannot distinguish "depth-hold off" from "depth-hold armed but temporarily suppressed by the heave auto-release" — both look identical as a `0` on the wire. `mavis_gamepad_node.py` is the only place that holds both pieces of state (`depth_hold_wanted` = operator's toggle intent, and the raw bit it's about to send), so it computes and publishes the 3-way status itself each control-loop tick, rather than trying to reconstruct it from Teensy telemetry. Do not map the dashboard's indicator directly to the raw `depth_hold` CSV field or to `/rov/system_mode` string-matching (e.g. checking for `"LOCK"` in the mode string) — the mode string never carries depth-hold state, and the raw bit flickers 1→0→1 on every heave touch by design (see the auto-release/re-lock note below). `/rov/depth_setpoint`, by contrast, genuinely only exists on the Teensy (it's the locked depth value from `depthSetpoint = depth` at the rising edge — see `main.cpp`), so it comes from `telemetry_receiver_node.py` parsing the `SPD:` token already present in the existing `P:` telemetry line — no firmware change was needed for that value.

### Video recording lives inside `video_receiver_node.py` — do not split it out

Recording is deliberately implemented inside `video_receiver_node.py` rather than as a separate subscriber node. A prior attempt at a standalone recorder node subscribing to `/rov/cam1|2/image_raw` caused the live video feed to disappear entirely: each 864×486 BGR frame is ~1.26 MB, and DDS sends a full independent copy over UDP loopback to every subscribing process, so a third subscriber pushed traffic past what the kernel socket buffers (~208 KB) and default MTU fragmentation could sustain — one dropped UDP fragment discards the whole BEST_EFFORT frame. See the module docstring in `video_receiver_node.py` for the full writeup. **Rule: never add a new process that subscribes to `image_raw`** — if you need a new image consumer, add it inside `video_receiver_node.py` or move it to a compressed topic.

Recordings are MJPG-in-AVI specifically because it's crash-safe (each frame stands alone; a process dying mid-write leaves a playable file), unlike MP4/H.264 which needs finalization. Output: `recordings/rov_cam{1,2}_<YYYYMMDD_HHMMSS>.avi`.

### Threading pattern used throughout the GCS nodes

`telemetry_receiver_node.py`, `video_receiver_node.py`, and `rov_dashboard_node.py` all follow the same rule: **never call `publisher.publish()` from a non-executor thread.** UDP/serial/GStreamer I/O happens on a background thread that only writes into a thread-safe buffer (`queue.Queue` or a locked "latest frame" slot); an actual ROS 2 `Timer` callback (which runs on the executor thread) drains the buffer and calls `publish()`. Follow this pattern for any new I/O-driven node — publishing directly from a raw `threading.Thread` causes intermittent DDS deadlocks that look like the feed "freezing then resuming."

### `data_logger_node.py`

Runs as its own node (not embedded in the dashboard) specifically so logging survives a GUI crash and has a lifecycle independent of Qt's event loop. Writes `logs/rov_log_<timestamp>.csv` (flushed per line, so it survives a hard crash) and a periodic/final `logs/rov_log_<timestamp>.json` snapshot. `surge_est` is derived locally from `TKIRI`/`TKANAN` PWM using the same formula as the dashboard's trajectory map — keep the two in sync if that formula changes. Also logs `depth_hold_status` and `depth_setpoint_m` (from `/rov/depth_hold_status` and `/rov/depth_setpoint`) for post-run analysis of depth-hold behavior.

### Non-ROS helper scripts (not part of the `rov_kki26` package)

- `MAVIS GAMEPAD/` — standalone gamepad test scripts (`cek_tombol.py`, `mavis_gamepad.py`), not wired into the ROS package or `setup.py` entry points.
- `Train Trejectory/` — `dataset_logger.py` / `train_velocity_model.py`, offline velocity-model training tooling, unrelated to the live GCS/ROS runtime.
- `test_qr.py` — standalone QR-decoding test script at the workspace root.
