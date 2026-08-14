# Carrier architecture

## Power tree

```text
J1 DC barrel input, 6S Li-ion battery, about 18-25.2 V
  -> F1 fuse -> TVS
     -> VIN_PROTECTED
        -> four JGA25-2430-CE motor power connectors, direct battery rail
        -> J2 screw terminal -> external 24 V to 5 V converter input

VIN_PROTECTED -> four JGA25-2430-CE motor power connectors
+5V       <- J3 screw terminal <- external 24 V to 5 V converter output
          -> remote Raspberry Pi harness, four servo connectors, ESP32 DevKit VIN/5V
          -> J41 screw terminal -> external L298N power terminals
3.3V      <- ESP32 DevKit 3V3 output; encoder and I2C pullups, IMU VCC
```

The 5 V converter is external and connects through two 5.08 mm screw-terminal
blocks: `J2` carries its 24 V input and `J3` receives its 5 V output. The
ESP32 DevKit's on-board regulator creates the low-current 3.3 V rail used by
the IMU, encoder pullups, and I2C pullups. Do not use that 3.3 V output for
motor or servo power.
The current placement sample does not yet include reverse-polarity
protection; add and verify that stage before connecting a battery.

The sample assumes a 6S Li-ion battery marketed as 24 V / 9 Ah. The motor rail
is not regulated: expect approximately 18-25.2 V over the discharge cycle.
The JGA25-2430-CE is specified as a 24 V motor, but the manufacturer does not
publish a maximum input voltage, so verify full-charge operation and motor
temperature before production. A 24 V buck-boost regulator can be added later
if constant speed or a guaranteed 24 V rail is required.

## ESP32-WROOM-32 DevKit pin map

The board uses a socket for the 38-pin ESP32-WROOM-32 DevKitC-style Type-C
board shown in the supplied photograph. It has a 25.40 mm header-row spacing
and 45.72 mm pad-to-pad span for each 19-pin row. The carrier reserves a
27.94 mm x 54.36 mm body keep-out and places the DevKit on the underside. The
mapping below assumes the official DevKitC header order with the USB connector
at the marked edge.

| Function | ESP32 GPIO | Carrier signal |
| --- | ---: | --- |
| Motor 1 PWM | 13 | `M1_PWM_3V3` direct to motor |
| Motor 2 PWM | 16 | `M2_PWM_3V3` direct to motor |
| Motor 3 PWM | 17 | `M3_PWM_3V3` direct to motor |
| Motor 4 PWM | 18 | `M4_PWM_3V3` direct to motor |
| Motor 1 direction | 23 | `M1_DIR_3V3` direct to motor |
| Motor 2 direction | 25 | `M2_DIR_3V3` direct to motor |
| Motor 3 direction | 26 | `M3_DIR_3V3` direct to motor |
| Motor 4 direction | 27 | `M4_DIR_3V3` direct to motor |
| Motor 1 encoder | 34 | `M1_ENC`, 3.3 V pull-up |
| Motor 2 encoder | 35 | `M2_ENC`, 3.3 V pull-up |
| Motor 3 encoder | 36 | `M3_ENC`, 3.3 V pull-up |
| Motor 4 encoder | 39 | `M4_ENC`, 3.3 V pull-up |
| Servo 1 PWM | 4 | `S1_PWM` |
| Servo 2 PWM | 14 | `S2_PWM` |
| Servo 3 PWM | 19 | `S3_PWM` |
| Servo 4 PWM | 21 | `S4_PWM` |
| L298N ENA / PWM | 22 | `GM_ENA_PWM_3V3` |
| L298N IN1 | 32 | `GM_IN1_3V3` |
| L298N IN2 | 12 | `GM_IN2_3V3` |
| I2C SDA | 5 | `I2C_SDA` |
| I2C SCL | 33 | `I2C_SCL` |
| Raspberry Pi UART TX | 1 | `PI_UART_TX_3V3` |
| Raspberry Pi UART RX | 3 | `PI_UART_RX_3V3` |

GPIO34-39 are input-only and are reserved for encoder signals. GPIO6-11 are
not used because they are connected to the ESP32 flash. GPIO5 is a strapping
pin; the I2C pull-up must remain high during reset. GPIO12 is used only for
L298N `IN2` and has a 10 k pull-down because it is a boot-configuration pin.

The carrier feeds the ESP32 socket with +5 V on VIN and uses the DevKit's
regulated 3.3 V output for encoder pull-ups, the IMU, and the I2C pull-ups.
The Pi's 3.3 V pin is not connected. The Pi UART is 3.3 V logic and shares
the DevKit UART0 pins; disconnect the Pi harness if the DevKit USB serial
adapter drives those pins during flashing.

## Connectors

### Motor connectors J10-J13

```text
1  VIN_PROTECTED (direct battery motor rail)
2  GND
3  PWM_3V3 (active-low, 15-25 kHz; direct ESP32 GPIO)
4  DIR_3V3 (direct ESP32 GPIO)
5  ENC     (open collector, 3.3 V pull-up on carrier)
```

The tested JGA25-2430-CE controller accepts the ESP32's 3.3 V GPIO directly,
so there is no level-shifter IC in this carrier. The PCB connects each ESP32
motor PWM and direction net directly to its motor connector. Motor black joins
the motor supply negative and the carrier `GND` plane.

PWM is active-low: GPIO high is the stopped state and GPIO low commands the
motor to run. For long motor cables, a 220-1000 ohm series resistor may be
added inline in a later revision. Never connect the white, orange, or yellow
signal wires to the 24 V rail.

### Servo connectors J20-J23

```text
1  +5V
2  GND
3  PWM     (ESP32 3.3 V logic, 50 Hz)
```

The four servos share the 5 V converter output. The final servo model must be
known before selecting the servo fuse and connector current rating.

### Raspberry Pi connector JPI1

```text
1  +5V
2  +5V
3  GND
4  GND
5  PI_UART_TX_3V3 (ESP32 TX -> Pi RX)
6  PI_UART_RX_3V3 (Pi TX -> ESP32 RX)
```

Use separate power and ground jumper wires sized for the Pi load. Do not
connect the Pi 3.3 V pin to the carrier.

### External buck terminals J2/J3

```text
J2 pin 1  VIN_PROTECTED -> buck IN+
J2 pin 2  GND           -> buck IN-
J3 pin 1  +5V           <- buck OUT+
J3 pin 2  GND           <- buck OUT-
```

### GM25-370 / L298N interface

```text
J40 (three-pin Dupont logic header)
1  GM_ENA_PWM_3V3 -> L298N ENA
2  GM_IN1_3V3     -> L298N IN1
3  GM_IN2_3V3     -> L298N IN2

J41 (three-position screw terminal)
1  +5V -> L298N motor-power terminal
2  GND -> L298N GND
3  +5V -> L298N 5V logic terminal
```

J40 uses a male Dupont header so it connects to the L298N's logic header with
female-to-female Dupont wires. J41 connects by screw-terminal wires to the
L298N's power terminal block. Connect the GM25-370 to L298N output A. Remove
the module's `5V_EN` jumper and feed its 5 V logic terminal from J41 pin 3.
The module is physically separate but not electrically isolated: its ground
must join carrier ground. The provided module is specified for 2 A maximum per
bridge, so verify the exact GM25-370 stall current and module temperature
before relying on it.

## Required next revision

- Add the full KiCad schematic and assign verified footprints.
- Verify the purchased 5 V converter pinout and thermal derating.
- Route high-current input, motor, and servo copper.
- Verify the remote Pi power connector, jumper wire gauge, and UART pin order.
- Add 470-1000 uF servo/5 V bulk capacitance and 470 uF Pi-side bulk capacitance.
- Test the external L298N at the exact GM25-370 stall current and replace it
  if voltage drop or temperature is excessive.
- Verify the four fitted 10 k encoder pull-ups and add input protection if
  cable length or the final encoder waveform requires it.
- Select the input fuse for the 6S battery, motor stall current, and 5 V
  converter input current; 15 A remains provisional until measured.
- Add 220 ohm series resistors on servo PWM and L298N logic signals.
- Add a hardware motor/servo power-enable or emergency-stop circuit.
