# Cubie Robot Runtime

C++ runtime for the Radxa Cubie A7S upper controller. The Cubie owns the
camera and high-level localization; an ESP32 owns the real-time motor, servo,
encoder, and IMU interfaces.

## Layout

```text
apps/       Executable entry points
config/     Versioned runtime calibration
include/    Public C++ interfaces grouped by subsystem
src/        Hardware, localization, planning, and control implementations
systemd/    Deployment units
tests/      Passive unit and replay tests
tools/      Linux control, capture, and calibration utilities
```

`dataset/`, `run-log/`, `videos/`, and `build/` are local working data and are
ignored by Git.

## Components

- `robotd`: sole owner of `/dev/ttyAS2`; proxies commands to the ESP32 through
  `/tmp/robotd.sock` and enforces a command heartbeat.
- `robotctl`: command-line client for `robotd`.
- `robotloc`: passive camera, wheel, and IMU localization logger. It never
  sends motion commands.
- `robot-runtime`: production name for the same passive runtime. Camera work
  and ESP32 telemetry sampling are decoupled so UART latency does not stall
  frame processing.
- `robot_transport`: UART framing and ESP32 transport library.
- `robot_location`: mecanum odometry, optical flow, ground projection, and
  fence particle-filter library.
- `robot_autonomy`: hardware-independent world model, solo mission state
  machine, and final command safety checks.

The production dependency direction is:

```text
hardware/perception -> localization -> planning -> control -> hardware
```

Perception publishes timestamped detections through the abstract `Detector`
interface. The YOLO26n backend will implement that interface after its model
artifact and Cubie NPU conversion are ready. Camera workers publish only the
newest frame; stale frames are intentionally discarded.

The ESP32 remains responsible for fast velocity PID, direction-change ramps,
and its independent 500 ms safety watchdog. Cubie localization must not be
placed in that control loop.

The Cubie safety supervisor independently rejects motion when localization is
stale or invalid, limits translation by vector magnitude, and reduces speed
when localization uncertainty is high. Hardware validation is never part of
the build or test suite.

## Build

The Cubie requires CMake, a C++20 compiler, and OpenCV development packages.
OpenCV 4.5.1 and OpenCV 5 are both supported.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$HOME/cubie-robot"
```

## Operate

Start the UART daemon before any client:

```sh
./build/robotd /dev/ttyAS2 /tmp/robotd.sock
./build/robotctl state
./build/robotctl imu
```

The principal drive command is:

```text
twist FORWARD_MPS LEFT_MPS YAW_RADPS
```

Other supported commands are `state`, `imu`, `telemetry`, `s3`, `ga25`,
`wheel`, `raw`, and `stop`. `robotd` refreshes active motion at 25 Hz and
sends `stop` if its client heartbeat expires after 250 ms.

For manual control over SSH, run this on the Cubie:

```sh
python3 tools/keyboard_control.py
```

`W/S` control forward motion, `A/D` strafe, and `Q/E` rotate. Space stops all
motion. Escape sends `stop` before exiting. Run the program in a real terminal
so key handling and emergency-stop behavior remain available.

## Localization

Run the passive logger from the repository root so its default configuration
path resolves correctly:

```sh
./build/robot-runtime --config config/robot.yaml \
  --log "run-log/location-$(date +%Y%m%d-%H%M%S).jsonl"
```

`config/robot.yaml` is the versioned source for UART/camera rates, camera
mount, arena dimensions, motion limits, detector paths, and servo envelopes.
The normal S3 range is 1600-2000 us; the firmware-only calibration envelope is
1550-2125 us. Command-line arguments override configuration values for
diagnostics.

The runtime reads only `state` and `telemetry`. Each JSONL frame contains the
telemetry sequence, age, and validity alongside wheel, IMU, optical-flow,
fence, and pose results. A temporary telemetry failure is logged as stale and
does not block camera capture. This process has no actuator command path.

It also sends the same passive JSON records over UDP port 3335. The configured
destination is the current debug host because the A314 access point filters
client broadcasts. Start the
host arena viewer after deploying and starting `robot-runtime` on the Cubie:

```sh
uv run --project tools python tools/robot_debug_gui.py
```

The GUI draws the fence particle filter in green/orange and independent
wheel-plus-relative-IMU dead reckoning in blue. This comparison exposes a bad
fence correction instead of hiding it inside a fused pose. Ranked visual-only fence matches are shown as
purple `V1..V4` markers with their mean wall residual in metres. Their search
runs at 2 Hz with a +/-25 degree relative-IMU yaw prior. The GUI draws a
two-sigma visual uncertainty ellipse. Its certainty combines the
absolute fence residual, separation from distant arena hypotheses, and the
local score-surface shape; it is a diagnostic score, not a calibrated
probability. The GUI uses UDP
only for localization display. Chassis commands travel over
an authenticated persistent SSH session to `robotctl --stream`. It opens
disarmed, Space always stops, closing an armed window sends `stop`, and it has
no servo or GA25 controls.

The default camera calibration is
`config/camera_fisheye_1280x720.yaml`. Override the starting pose with
`--initial-x`, `--initial-y`, and `--initial-yaw`. Use `--global-initialize`
only when no approximate start is known.

The current estimator is suitable for broad navigation such as reaching the
field center. A single visible wall does not constrain position along that
wall, so precise home docking requires an active view of a corner or two
nonparallel walls. The particle-filter uncertainty is not yet a certified
safety bound.

Full-resolution rectification and MJPEG logging reduce the observed Cubie loop
rate to about 6.4 Hz. Use `--no-video` when localization throughput matters.
The ESP32 continues its control loop independently at all times.

## Solo Runtime

The autonomous mission progresses through `boot`, `self_test`, `localize`,
`search_target`, `approach_target`, `acquire_target`, `navigate_home`, and
`deposit`. Loss of localization enters `recover_localization`; a hardware or
runtime fault enters the latched `safe_stop` state.

Initial integration stops after approaching a target and returning home.
Servo and GA25 actions remain disabled until navigation has passed logged
replay and deliberate low-speed arena validation.

## ESP32 UART OTA

The firmware on the `esp32` branch has two OTA application partitions. The
UART updater sends `ota SIZE CRC32`, writes and verifies the inactive
partition, selects it, and reboots automatically. Firmware stops its motors
and releases the servo before accepting image bytes.

Only one process may own `/dev/ttyAS2`, so stop `robotd` before updating:

```sh
sudo systemctl stop robotd
python3 tools/uart_ota_update.py firmware.bin --port /dev/ttyAS2
sudo systemctl start robotd
```

OTA is an explicit maintenance operation and is never called by startup,
builds, or tests. Its UART format is not authenticated, so use only trusted
firmware images on the trusted Cubie.

## Camera Tools

The Python environment is managed independently from the C++ build:

```sh
uv sync --project tools
```

- `tools/capture_video.py`: continuous local video segments, rectified by
  default.
- `tools/camera_preview.py`: host-side SSH preview and frame capture.
- `tools/capture_fisheye_calibration.py`: collect diverse raw checkerboard
  views.
- `tools/calibrate_fisheye_intrinsics.py`: generate a fisheye calibration.
- `tools/capture_arena_poses.py`: collect measured arena poses for offline
  localization calibration.

Example arena capture on the Cubie:

```sh
python3 tools/capture_arena_poses.py \
  --output-dir "$HOME/arena-calibration/session-$(date +%Y%m%d-%H%M%S)"
```

Enter `x_m y_m yaw_deg [note]` for each stationary pose. The arena frame uses
`+x` forward from home along the 3 m side, `+y` left along the 1.985 m side,
and positive yaw toward `+y`.

## Offline Cubie Benchmark

`robot-runtime` accepts a recorded rectified AVI for passive benchmarking:

```sh
./build/robot-runtime --camera RUN.avi --rectified-input --no-video \
  --no-broadcast --socket /tmp/no-robotd --max-frames 300
```

The resident `src/a733-yolo26/yolo26_video_benchmark` keeps the NPU network
loaded and reports in-memory preprocessing, inference, and decode/NMS timing.
On the 2026-09-03 `location-20260903-065016` run, 300-frame concurrent tests
measured 3.83 FPS for exhaustive fence localization and 51.2 FPS for YOLO26.
The visual geometry grid search is currently the limiting stage.

## Service

Install `systemd/robotd.service` as `/etc/systemd/system/robotd.service`, then:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now robotd
```

The service expects the installed binary at
`/home/radxa/cubie-robot/bin/robotd`. The `radxa` user must remain in the
`dialout` group, and no second process may open `/dev/ttyAS2`.
