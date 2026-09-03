# Cubie Robot Runtime

C++ runtime for the Radxa Cubie A7S upper controller. The Cubie owns the
camera and high-level localization; an ESP32 owns the real-time motor, servo,
encoder, and IMU interfaces.

## Layout

```text
apps/       Executable entry points
config/     Versioned runtime calibration
include/    Public C++ interfaces
src/        Reusable C++ implementations
systemd/    Deployment units
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
- `robot_transport`: UART framing and ESP32 transport library.
- `robot_location`: mecanum odometry, optical flow, ground projection, and
  fence particle-filter library.

The ESP32 remains responsible for fast velocity PID, direction-change ramps,
and its independent 500 ms safety watchdog. Cubie localization must not be
placed in that control loop.

## Build

The Cubie requires CMake, a C++20 compiler, and OpenCV development packages.
OpenCV 4.5.1 and OpenCV 5 are both supported.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
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
./build/robotloc \
  --log "run-log/location-$(date +%Y%m%d-%H%M%S).jsonl"
```

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

## Service

Install `systemd/robotd.service` as `/etc/systemd/system/robotd.service`, then:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now robotd
```

The service expects the installed binary at
`/home/radxa/cubie-robot/bin/robotd`. The `radxa` user must remain in the
`dialout` group, and no second process may open `/dev/ttyAS2`.
