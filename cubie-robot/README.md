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

To capture calibrated 1280x720 video on the Cubie, with no preview or network
transfer, use the local wrapper. It records consecutive 30-second segments
until `Ctrl-C`:

```sh
python3 tools/capture_video.py
```

Files are named `capture-<time>-0001.avi`, `capture-<time>-0002.avi`, and so on.
Use `--duration` to change the length of each segment.

The Camera1 fisheye calibration is included under `calibration/`; the C++
OpenCV recorder is compiled locally on the Cubie when needed. For the faster
native 640x480 MJPEG recorder, use `--raw`.

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
