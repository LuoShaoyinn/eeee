# ESP32 JGA25-2430-CE Motor Test

ESP-IDF MCPWM test application for a JGA25-2430-CE motor with an integrated controller.

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

## Motor wiring

Use an external regulated motor supply matching the motor label, either 12 V or 24 V. Do not power the motor from the ESP32.

| Motor wire | Connection |
| --- | --- |
| Red | External motor supply positive |
| Black | External motor supply negative and ESP32 GND |
| White | ESP32 GPIO25, 20 kHz speed PWM |
| Orange | ESP32 GPIO26 for direction, or GND for a fixed direction |
| Yellow | Optional pulse feedback input; use a 3.3 V pull-up |

For speed-only control, connect **red**, **black**, and **white**, and connect **orange to black/GND** to select a fixed direction. Keep the grounds common. Do not leave orange floating. Leave yellow disconnected unless measuring speed.

The firmware starts at 0% duty for safety. Set `MOTOR_INITIAL_DUTY_PERCENT` in `main/motor_test_main.c` to a small value such as 10, rebuild, and flash. Keep the shaft unloaded and use a current-limited supply for the first test.

## Flash and monitor

Flash the connected board:

```bash
idf.py -p /dev/ttyUSB0 flash
```

Open the serial monitor at 115200 baud:

```bash
idf.py -p /dev/ttyUSB0 monitor
```

The monitor should show the motor test configuration. Press `Ctrl-]` to exit.

## Servo test

The `test/servo` branch selects the reusable `servo-driver/standard-3wire` component and runs a conservative sweep on GPIO27. The servo needs a separate regulated 5-6 V supply, with servo black and ESP32 GND connected together. Servo yellow connects to GPIO27; servo red must not connect to an ESP32 3.3 V pin.

The test uses 50 Hz PWM and sweeps 1200-1800 us. A standard three-wire servo provides no position feedback, so the driver can report only the commanded pulse.

## Project layout

- `CMakeLists.txt`: ESP-IDF project definition
- `main/motor_test_main.c`: motor application-level test harness
- `main/servo_test_main.c`: servo application-level test harness
- `main/CMakeLists.txt`: component registration
- `motor-driver/JGA25-2430-CE/`: reusable motor-driver component
- `motor-driver/JGA25-2430-CE/include/jga25_2430_ce.h`: public driver API
- `motor-driver/JGA25-2430-CE/src/jga25_2430_ce.c`: MCPWM implementation
- `motor-driver/JGA25-2430-CE/README.md`: component-specific wiring and ratings
- `servo-driver/standard-3wire/`: reusable three-wire servo component
- `servo-driver/standard-3wire/include/standard_servo.h`: public servo API
- `servo-driver/standard-3wire/src/standard_servo.c`: LEDC implementation
- `servo-driver/standard-3wire/README.md`: servo wiring and API documentation
- `sdkconfig.defaults`: shared default configuration
- `.gitignore`: generated ESP-IDF output exclusions

## KiCad carrier board

The `test/kicad-eda` branch contains the first integrated carrier-board sample
for Raspberry Pi 4B, ESP32-WROOM-32 DevKit, four JGA25-2430-CE motors, and four
servos:

- `hardware/robot-carrier/robot-carrier.kicad_pcb`: placement prototype
- `hardware/robot-carrier/carrier-architecture.md`: power tree, pin map, and connector wiring
- `hardware/robot-carrier/README.md`: converter selection, safety notes, and LCSC production workflow

The carrier routes the tested JGA25-2430-CE PWM and direction signals directly
from ESP32 3.3 V GPIOs. Each active-low PWM input and each open-collector
encoder input has a 10 k pull-up to 3.3 V; do not pull these signals up to the
5 V or motor rail.
