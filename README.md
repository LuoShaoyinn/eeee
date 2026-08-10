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

## Project layout

- `CMakeLists.txt`: ESP-IDF project definition
- `main/motor_test_main.c`: small application-level test harness
- `main/CMakeLists.txt`: component registration
- `motor-driver/JCA25-2430-CE/`: reusable motor-driver component
- `motor-driver/JCA25-2430-CE/include/jga25_2430_ce.h`: public driver API
- `motor-driver/JCA25-2430-CE/src/jga25_2430_ce.c`: MCPWM implementation
- `motor-driver/JCA25-2430-CE/README.md`: component-specific wiring and ratings
- `sdkconfig.defaults`: shared default configuration
- `.gitignore`: generated ESP-IDF output exclusions
