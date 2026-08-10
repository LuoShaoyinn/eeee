# ESP32 Hello World

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
