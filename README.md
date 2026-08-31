# ESP32 Robot Controller

Four-wheel JGA25-2430-CE production controller for the carrier board. It uses active-low 20 kHz PWM, each motor's direction line, and the yellow FG speed output.

| Wheel | PWM | Direction | FG |
|---|---:|---:|---:|
| M1/front-left | GPIO23 | GPIO32 | GPIO36 |
| M2/front-right | GPIO21 | GPIO25 | GPIO39 |
| M3/rear-left | GPIO26 | GPIO27 | GPIO35 |
| M4/rear-right | GPIO19 | GPIO13 | GPIO34 |

The carrier supplies the required 3.3 V pull-ups for FG. FG is single-channel speed magnitude, not quadrature: it cannot prove direction. The firmware uses it for PI speed feedback and zero-speed confirmation before reversal; commanded DIR supplies the sign. Parameters are 18 FG pulses/output revolution, 620 RPM no-load, and 450 RPM rated-load. The wheel controller runs every 20 ms and updates its PCNT-derived RPM/PID feedback every 60 ms. The controller starts stopped, ramps duty, and stops after 500 ms without a valid command.

Chassis values: 190 mm wheel-center square and 23 mm wheel radius. The mirrored right-side mounts use inverted electrical direction by default (M2 and M4). Validate each wheel one at a time and correct `invert_direction` in `main/motor_control_main.c` for the actual Mecanum roller handedness/wiring. The FG source emits 9 pulses per motor-shaft revolution and the 9.6:1 gearbox yields 86.4 FG edges per output-shaft revolution. `encoder_pulses_per_output_rev` accepts fractional values for this reason.

`ROBOT_ENABLE_WIFI` in `main/motor_control_main.c` is `0` by default, so the
robot starts with its Wi-Fi radio and UDP listener disabled. The Cubie UART0
link is therefore the sole runtime control interface. Set the switch to `1`
to restore the retained AP/station and UDP control path; then copy
`main/wifi_credentials.h.example` to ignored local `main/wifi_credentials.h`.
An ESP32-WROOM-32 supports 2.4 GHz Wi-Fi only.

```sh
source /opt/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

With the host connected to the ESP32 fallback AP:

```sh
python3 tools/keyboard_drive.py --host 192.168.4.1
```

UDP port `3333` accepts `drive FORWARD STRAFE TURN` with values from `-1.0` to `1.0`, `wheel MOTOR SPEED`, `stop`, `telemetry`, or `pid [KP KI]`. The PID query returns active gains; setting gains resets integrators immediately. The telemetry response contains per-wheel target, PCNT-derived RPM, latest FG edges, cumulative PCNT edges, duty, and reversal state. Subtract two `total` readings over a known interval to inspect hardware-counter edge rate directly.

Drive changes use a bounded PWM ramp. A regular `stop` coasts down at the configured deceleration rate. A direction reversal first ramps PWM to zero, waits for the FG signal to become quiet and observes the reversal dwell, then changes the direction line. A lost UDP command stream remains an immediate coast-to-stop safety action.

Minimal ESP-IDF application for an ESP32. The application prints `Hello, ESP32!` once per second over the default console UART.

## Hardware

The connected device was detected as:

- Chip: ESP32-D0WDQ6
- Revision: v1.1
- Features: Wi-Fi, Bluetooth, dual core
- Flash: 4 MB
- USB-UART port: `/dev/ttyUSB0`

The USB-UART bridge is a WCH CH340 (`1a86:7523`).

## Remote IMU debug

The carrier board's `J30` IMU header uses I2C. Connect JY61P `VCC` to `3V3`,
`GND` to `GND`, `SCL` to ESP32 `GPIO5`, and `SDA` to ESP32 `GPIO33`.
The firmware reads the module's default I2C address `0x50` at 100 kHz. Do not
also connect the JY61P UART pins for this integration.

From any device on the ESP32 Wi-Fi network, send UDP `imu` to port `3333`.
The reply reports the latest acceleration, angular rate, roll/pitch/yaw, frame
count, transaction-error count, and data age. A reply of `waiting for JY61P
I2C addr 0x50` indicates a wiring, supply, pull-up, or address/mode problem.

## S3 remote debug

S3 is a standard positional MG995 on GPIO17. At boot it commands 0 degrees
(1000 us); after `stop`, PWM is disabled so it does not hold a blocked linkage.
`s3 ANGLE` commands a slow one-degree-per-50-ms move over 0-150 degrees;
`s3 center` commands 60 degrees. `s3 release` stops PWM and releases holding
torque immediately. The MG995 has no angle readback: remove or align the horn
with the linkage at the intended center while unpowered before the first
installed test.

Send `s3` without an argument to inspect the commanded state. The reply shows
the most recently applied angle, target angle, 1,000-2,000 us pulse width,
50 Hz duty percentage, and the exact 16-bit LEDC duty count. After `s3
release`, the reported output duty is zero even though the last commanded angle
is retained for reference.

Run `python3 tools/keyboard_drive.py --s3` on the host for a guarded manual
test: `A`/`D` adjust in 5-degree steps, `0` commands 0 degrees, `C` commands
60 degrees, and Space or Escape releases the servo.

Watch IMU telemetry over Wi-Fi with `python3 tools/keyboard_drive.py --imu`;
it keeps retrying until stopped with Ctrl-C, so it can be started before the
ESP32 powers on.

## Cubie UART debug and control

The carrier `JPI1` UART is ESP32 UART0 at 115200 baud, 8N1, with the 470 ohm
series resistors already fitted. Its connection is crossed: carrier
`PI_UART_TX_3V3` goes to Cubie RXD; carrier `PI_UART_RX_3V3` goes to Cubie TXD;
grounds join. Do not connect either 5 V or 3.3 V supply between boards.

Send one ASCII command per newline. UART accepts all production controls:
`state`, `imu`, `telemetry`, `drive F S T`, `twist VX_MPS VY_MPS WZ_RADPS`,
`wheel M SPEED`, `raw M DUTY`, `pid [KP KI]`, `s3 ANGLE`, `s3 center`,
`s3 release`, `ga25 DUTY`, and `stop`. Each command reply starts with `@ `.
For example, send `imu\n` to read the latest IMU data, or `s3 60\n` to move
S3 slowly to center. `s3 release\n` and `stop\n` release S3 immediately.

`state` is the compact control-loop snapshot: timestamp, IMU age/gyro/angle,
four wheel RPM/FG samples, and GA25 state. `twist` is the Cubie physical-unit
body command. It is conservatively limited to `+/-0.40 m/s` forward/strafe and
`+/-2.00 rad/s` yaw pending chassis calibration. The legacy normalized `drive`
command remains for bench tests.

The [Cubie C++ transport](cubie-robot/README.md) owns `/dev/ttyAS2` through
the enabled `robotd.service`. Its `robotctl` client communicates through a
local Unix socket rather than opening the serial port directly. `robotd`
refreshes active twists at 25 Hz, stops after 250 ms without client refreshes,
and relies on the ESP32's independent 500 ms watchdog as the second stop layer.

`raw M DUTY` is a single-JGA25 wheel wiring test which bypasses PCNT/encoder feedback
and PID. `M` is connector 1 through 4 and `DUTY` is -100 through 100. Refresh
the command within 500 ms while it should run; otherwise the firmware stops it.

The separate 6 V GA25-370 driving motor is controlled through the external
L298N, not `raw M DUTY`. Keep the L298N `ENA` jumper fitted. On the carrier's
`J40` header, GPIO12 (physical pin 3, 10 kOhm pulldown) drives L298N `IN1` and
GPIO16 (physical pin 1, 10 kOhm pulldown) drives L298N `IN2`. GPIO14 (physical
pin 2, no pulldown) is the single-channel GA25 encoder input. The PCB's old
`ENA`/`IN1`/`IN2` silkscreen labels therefore describe the physical traces, not
the new external L298N connection order. Never connect J40 pin 1 to L298N
`ENA`, and do not remove the L298N `ENA` jumper. The encoder must be a 3.3 V
safe signal before it reaches GPIO14.

Use `ga25 DUTY` for signed open-loop PWM. `ga25` reports the latest hardware
counter sample and cumulative encoder edges; no GA25 encoder PID is enabled
until its pulses-per-output-revolution is measured. Run `python3 tools/keyboard_drive.py --ga25 --host
192.168.19.137` for an interactive test; A/D change PWM by 5%, and Space/Esc
ramps to a stop.

### GA25 bench result

The external L298N and GA25-370 were verified with the above reassigned wiring.
Positive `ga25` duty produces a 20 kHz PWM waveform at the L298N motor output;
the motor-side average voltage is duty-scaled and includes the L298N bridge
drop, so it is lower than the 5 V supply. At 25% duty the measured average was
about 0.6 V, which was insufficient to reliably overcome static friction. The
motor started consistently at 80% duty.

The encoder on GPIO14 was counted by ESP32 PCNT hardware with a 10 us glitch
filter. With the mechanism free, five-second count windows measured:

| Command duty | Encoder edges | Window | Edge rate |
|---:|---:|---:|---:|
| 60% | 1,545 | 5.37 s | 288 edges/s |
| 80% | 2,633 | 5.34 s | 493 edges/s |

This confirms the encoder signal and the count rate scale with motor command.
It does not establish output RPM: measure encoder pulses per output revolution
before converting these counts into speed or enabling closed-loop GA25 control.
Always issue `ga25 0` or `stop` after a bench run; the command watchdog also
ramps GA25 to zero after 500 ms without refreshes.

## UART application updates

`partitions.csv` reserves two 1.875 MiB application slots. This enables updates
over the existing Cubie UART link after a one-time manual migration. The ESP32
ROM downloader cannot be entered from application code on this DevKit because
GPIO0 is only sampled during reset.

For the one-time migration, put the ESP32 in download mode with the physical
BOOT/EN buttons and flash the bootloader, partition table, and application in
one operation. Subsequent updates need no button press:

```sh
python3 tools/uart_ota_update.py build/esp32_hello_world.bin --port /dev/ttyAS2
```

The updater sends `ota SIZE CRC32` over UART0, writes the inactive slot, checks
the transferred image CRC32, marks that slot for the next boot, and reboots.
It stops the chassis, releases S3, and stops GA25 before accepting an image.
CRC32 protects against accidental serial corruption; it is not a signed or
authenticated update format, so only run the updater from the trusted Cubie.

## Environment

This project was built with ESP-IDF v6.0.2 on Arch Linux. Load the ESP-IDF toolchain before using `idf.py`:

```bash
source /opt/esp-idf/export.sh
```

## Build

Configure the target and build:

```bash
idf.py set-target esp32
idf.py build
```

The application binary is generated at `build/esp32_hello_world.bin`.

## Flash and monitor

Flash the connected board:

```bash
idf.py -p /dev/ttyUSB0 flash
```

Open the serial monitor at 115200 baud:

```bash
idf.py -p /dev/ttyUSB0 monitor
```

The monitor should show repeated `Hello, ESP32!` lines. Press `Ctrl-]` to exit.

## Project layout

- `CMakeLists.txt`: ESP-IDF project definition
- `main/hello_world_main.c`: application entry point
- `main/CMakeLists.txt`: component registration
- `sdkconfig.defaults`: shared default configuration
- `.gitignore`: generated ESP-IDF output exclusions
