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

For a suspended-car test, use a bounded heartbeat. It stops automatically when
the command exits:

```sh
./build/robotctl --hold-ms 2000 twist 0.10 0.00 0.00
./build/robotctl stop
```

The physical `twist` limits are conservative pending chassis calibration:
`|vx|, |vy| <= 0.40 m/s` and `|yaw_rate| <= 2.00 rad/s`.

For automatic startup, install `systemd/robotd.service` as
`/etc/systemd/system/robotd.service`, run `systemctl daemon-reload`, then
`systemctl enable --now robotd`. The unit runs as `radxa`, which must remain in
the `dialout` group. Stop any manually started `robotd` before enabling the
service because exactly one process may own `/dev/ttyAS2`.
