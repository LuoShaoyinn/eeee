# ESP32 Robot Controller

Four-wheel JGA25-2430-CE production controller for the carrier board. It uses active-low 20 kHz PWM, each motor's direction line, and the yellow FG speed output.

| Wheel | PWM | Direction | FG |
|---|---:|---:|---:|
| M1/front-left | GPIO23 | GPIO32 | GPIO36 |
| M2/front-right | GPIO21 | GPIO25 | GPIO39 |
| M3/rear-left | GPIO26 | GPIO27 | GPIO35 |
| M4/rear-right | GPIO19 | GPIO13 | GPIO34 |

The carrier supplies the required 3.3 V pull-ups for FG. FG is single-channel speed magnitude, not quadrature: it cannot prove direction. The firmware uses it for PI speed feedback and zero-speed confirmation before reversal; commanded DIR supplies the sign. Parameters are 18 FG pulses/output revolution, 620 RPM no-load, and 450 RPM rated-load. Normalized commands use 620 RPM as their full-scale target; 450 RPM is the expected sustained loaded maximum. The controller starts stopped, limits duty to 50%, ramps duty, and stops after 300 ms without a valid command.

Chassis values: 190 mm wheel-center square and 23 mm wheel radius. The mirrored right-side mounts use inverted electrical direction by default (M2 and M4). Validate each wheel one at a time and correct `invert_direction` in `main/motor_control_main.c` for the actual Mecanum roller handedness/wiring. The FG source emits 9 pulses per motor-shaft revolution and the 9.6:1 gearbox yields 86.4 FG edges per output-shaft revolution. `encoder_pulses_per_output_rev` accepts fractional values for this reason.

Copy `main/wifi_credentials.h.example` to ignored local `main/wifi_credentials.h`. The ESP32 starts a 2.4 GHz station and fallback AP `robot-esp32` at `192.168.4.1`. An ESP32-WROOM-32 cannot join a 5 GHz-only network.

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
