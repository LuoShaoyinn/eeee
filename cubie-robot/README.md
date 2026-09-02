# Cubie Robot Transport

`robotd` is the sole owner of the Cubie A7S UART link to the ESP32. It serves
newline-delimited commands over `/tmp/robotd.sock`. Replies are returned as
single text lines. ESP32 boot logs are ignored; only replies prefixed by `@ `
are accepted by the serial client.

`twist VX_MPS VY_MPS WZ_RADPS` is the control command. `robotd` refreshes an
active twist at 25 Hz, then sends `stop` after 250 ms without a client refresh.
The ESP32 retains its independent 500 ms watchdog. Other supported commands
are passed through directly: `state`, `imu`, `telemetry`, `s3 ANGLE`,
`s3 release`, `ga25 DUTY`, `wheel M SPEED`, `raw M DUTY`, and `stop`.

Build on the Cubie:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --install build --prefix "$HOME/cubie-robot"
```

Run a daemon and issue status commands:

```sh
./build/robotd /dev/ttyAS2 /tmp/robotd.sock
./build/robotctl state
./build/robotctl imu
```

For manual keyboard control over SSH, run the Python client on the Cubie. It
talks only to `robotd`'s Unix socket, so it never competes for the UART:

```sh
python3 tools/keyboard_control.py
```

Each `W/S` press adds/subtracts forward speed, each `A/D` press adds/subtracts
lateral speed, and each `Q/E` press adds/subtracts yaw rate. The defaults cap
linear speed at 0.60 m/s and yaw at 2.00 rad/s; adjust the increments or limits
with `--linear`, `--yaw`, `--max-linear`, and `--max-yaw`. `Space` stops all
ESP32-controlled motion. `X`, `Z`, and `C` clear forward, lateral, and yaw
components independently. `[`/`]` adjust S3 within its default active
calibration range of 1600--2000 us. `R` releases S3/no PWM signal. `,`/`.` adjust
GA25 speed, `F`/`B` select GA25 forward/reverse direction, `G` stops GA25,
and `T` prints a state snapshot. Direction selection at zero GA25 speed does
not start the motor. Escape issues `stop` before exiting.

Use `--servo-active-min-pulse`, `--servo-max-pulse`, and
`--servo-pulse-step` to set the keyboard calibration range. Active values must
remain within the ESP32 firmware's accepted S3 pulse range.
=======
For a Windows keyboard client, use tools/keyboard_control_windows.py. It uses
Windows' built-in msvcrt keyboard API and a persistent OpenSSH channel; on the
Cubie, a short-lived Python bridge forwards each command to the local Unix
socket. No robot control port is exposed on the network.

The Windows PC needs Python 3 and the Windows OpenSSH Client. The Cubie must
already have robotd running and accept public-key (or SSH-agent)
authentication: the client intentionally uses BatchMode=yes so it cannot
silently hang waiting for a password. First validate the SSH setup:

~~~powershell
ssh radxa@192.168.1.112 "test -S /tmp/robotd.sock && echo robotd-ready"
~~~

Then run the controller from PowerShell or Command Prompt:

~~~powershell
python .\tools\keyboard_control_windows.py --host 192.168.1.112 --user radxa
~~~

Use --linear 0.05 --yaw 0.25 --max-linear 0.20 --max-yaw 1.00 for a
low-speed first test. --socket and --remote-python select non-default paths
or Python commands on the Cubie. This program must be run in a real Windows
console; IDE output panes generally do not provide msvcrt keyboard input. Its
controls and safety behaviour are identical to the Linux client.

Password SSH login is also available with --auth password. It requires the
Paramiko package and prompts securely when the program starts; do not place a
password in a source file or command line:

~~~powershell
python -m pip install paramiko
python .\tools\keyboard_control_windows.py --host 192.168.19.105 --user radxa --auth password
~~~

The Cubie's host key must already be trusted by the Windows account. Complete
one normal SSH login first and accept its host-key prompt when asked.

To capture calibrated 1280x720 video on the Cubie, with no preview or network
transfer, use the local wrapper. It records consecutive 30-second segments
until `Ctrl-C`:

```sh
python3 tools/capture_video.py
```

Files are named `capture-<time>-0001.avi`, `capture-<time>-0002.avi`, and so on.
Use `--duration` to change the length of each segment.

## Passive visual localization logger

`robotloc` is a passive estimator: it reads `/dev/video0` and the existing
`robotd` Unix socket, but never sends `twist`, `wheel`, `raw`, `ga25`, or servo
commands. It rectifies the fisheye camera, estimates short-term ground motion
from tracked floor features, converts signed ESP32 wheel RPM to mecanum body
velocity, integrates IMU Z gyro rate, and weights a particle filter against
the blue lower-fence observations. One JSON object is written per frame. It
also writes an MJPEG AVI of the 1280x720 rectified frames; `frame_index` in the
JSONL is the corresponding AVI frame and `monotonic_ns` preserves capture
timing. By default the AVI has the same path as the JSONL with `.avi` instead
of `.jsonl`; use `--video PATH` to override it or `--no-video` for telemetry
only.

Build it alongside the other Cubie programs, then run it locally on the Cubie:

```sh
cmake -S . -B build-location -DCMAKE_BUILD_TYPE=Release
cmake --build build-location -j2
./build-location/robotloc --log logs/location-$(date +%Y%m%d-%H%M%S).jsonl
```

It defaults to the current fitted rigid-mount values: 0.12910 m camera height,
30.0296 degree downward pitch, and 0.2071 degree roll. It also defaults to a
known start near the home corner: `(x, y, yaw) = (0.10 m, 0.10 m, 0 deg)`.
Override those with `--initial-x`, `--initial-y`, and `--initial-yaw`; use
`--global-initialize` only when no start pose is known. `--max-frames N` is
useful for a bounded smoke test; `Ctrl-C` ends a normal logging run.

The current particle likelihood is intentionally conservative. It trims fence
points that do not agree with the rectangular 3.0 m by 1.985 m field, but a
blue robot or incomplete wall view can still create ambiguous global poses.
Treat this as a logged localization experiment until it has been evaluated
against measured robot poses.

The Camera1 fisheye calibration is included under `calibration/`; the C++
OpenCV recorder is compiled locally on the Cubie when needed. For the faster
native 640x480 MJPEG recorder, use `--raw`.

## Fixed camera height and pitch

Use tools/calibrate_camera_mount.py when the camera is rigidly mounted on the
car and the required result is its optical-centre height and optical-axis
pitch. It uses the existing rectified camera intrinsics, so images must be the
1280x720 undistorted output.

Park the car on level ground. Place the checkerboard flat on the same ground,
with no visible curl or tilt, and take at least three images from different
checkerboard positions or distances without moving the car. The board plane is
the height reference; if the printed checkerboard is raised above the ground,
measure that offset and pass it as --plane-height-m. Pitch is reported in
degrees, with zero meaning horizontal and a positive value meaning the optical
axis points downward.

For a 9 by 6 inner-corner board with 24 mm squares:

~~~sh
python3 tools/calibrate_camera_mount.py \
  --images calibration-session/floor-*.png \
  --pattern-cols 9 --pattern-rows 6 --square-m 0.024 \
  --plane-height-m 0.001 \
  --output calibration/camera1_mount.yaml
~~~

The YAML records the mean and standard deviation. Large standard deviation
usually means the floor/board was not level, the vehicle moved, a fisheye
image was supplied instead of a rectified image, or the board dimensions were
entered incorrectly.

## Rectified pixel to car-ground coordinates

`tools/pixel_to_car_frame.py` projects a pixel from the rectified 1280x720
camera image onto the level ground. The result is in metres, with the origin at
the ground point directly below the camera optical centre: `+forward` points in
front of the car and `+left` points to the car's left.

~~~sh
python3 tools/pixel_to_car_frame.py 640 500
~~~

It uses `camera1_mount.yaml`'s rectified intrinsics,  optical-centre height,
and downward pitch. It deliberately rejects pixels on or above the ground
horizon. This initial transform assumes the camera is mounted with zero roll
and its optical axis aimed straight ahead in the vehicle yaw direction. Use the
camera--IMU extrinsic calibration below, plus the IMU orientation, before using
it while the car rolls, pitches, or yaws.

## Camera--IMU extrinsics from a checkerboard

For already rectified 1280x720 images, use
tools/calibrate_camera_imu.py. It keeps the existing rectified_K as the camera
intrinsics and uses zero distortion; it does not recalibrate the raw fisheye
model. It estimates the rigid rotation from IMU coordinates to camera
coordinates with OpenCV hand-eye calibration.

The calibration board must remain still in the room. Move the rigid
camera--IMU assembly through at least 12 static, synchronised poses while the
board remains fully visible. Include substantial roll, pitch, and yaw changes
(at least 20 degrees of total rotation), and use an IMU orientation with an
observable yaw reference. Gravity-only roll/pitch cannot determine the full
three-dimensional rotation.

Save each rectified image and its simultaneous IMU orientation in one CSV row:

~~~text
image,qw,qx,qy,qz
images/0001.png,0.9981,0.0102,-0.0450,0.0400
images/0002.png,0.9650,-0.0301,0.1840,0.1840
~~~

The quaternion order is scalar first, and it must map an IMU-coordinate vector
to the fixed world frame: p_world = R_world_from_imu * p_imu. Images must be
the same 1280x720 rectified output used by the existing calibration. Confirm
the exact output convention of the ESP32 imu command before converting it to
this CSV; the tool deliberately does not guess a firmware-specific IMU text
format.

For example, a 9 by 6 inner-corner board with 24 mm squares is calibrated by:

~~~sh
python3 tools/calibrate_camera_imu.py \
  --imu-csv calibration-session/imu.csv \
  --images calibration-session \
  --pattern-cols 9 --pattern-rows 6 --square-m 0.024 \
  --imu-to-camera-m 0.035 0.010 -0.012 \
  --output calibration/camera1_imu_extrinsics.yaml
~~~

The X, Y, Z values are the measured signed vector from the IMU centre to the
camera optical centre, expressed in the IMU coordinate axes and in metres.
Passing only --imu-camera-distance-m records the measured centre distance, but
intentionally leaves translation unresolved: a scalar distance alone cannot
determine its three-dimensional direction. The generated YAML contains the
transform convention p_camera = R_imu_to_camera * p_imu + t_imu_to_camera,
reprojection error, rotation residual, and orientation span.

For a suspended-car test, use a bounded heartbeat. It stops automatically when
the command exits:

```sh
./build/robotctl --hold-ms 2000 twist 0.10 0.00 0.00
./build/robotctl stop
```

The physical `twist` limits are conservative pending chassis calibration:
`|vx|, |vy| <= 0.60 m/s` and `|yaw_rate| <= 2.00 rad/s`.

For automatic startup, install `systemd/robotd.service` as
`/etc/systemd/system/robotd.service`, run `systemctl daemon-reload`, then
`systemctl enable --now robotd`. The unit runs as `radxa`, which must remain in
the `dialout` group. Stop any manually started `robotd` before enabling the
service because exactly one process may own `/dev/ttyAS2`.
