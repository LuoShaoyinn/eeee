# JY61P UART Driver

Reusable ESP-IDF component for WitMotion JY61P/JY901-family sensors using the
standard 11-byte UART protocol at 9600 8N1 by default.

## Wiring

| Sensor | ESP32 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| TX | configured RX GPIO |
| RX | configured TX GPIO |

The application test uses UART1 TX GPIO22 and RX GPIO21. TX and RX are crossed.

## API

`jy61p_uart_init()` installs and configures the UART. Set
`configure_magnetic_output` to true to send the documented configuration
sequence that enables acceleration, gyro, angle, magnetic, and port frames.
`jy61p_uart_read()` synchronizes on `0x55`, validates the checksum, and decodes
acceleration, angular velocity, and angle units.
