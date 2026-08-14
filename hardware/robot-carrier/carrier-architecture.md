# Carrier architecture

## Power tree

```text
J1 DC barrel input, 6S Li-ion battery, about 18-25.2 V
  -> F1 fuse -> TVS
     -> VIN_PROTECTED
        -> four JGA25-2430-CE motor power connectors, direct battery rail
        -> U6: URF2405QB-100WR3 -> +5V, 20 A / 100 W

VIN_PROTECTED -> four JGA25-2430-CE motor power connectors
+5V       -> Raspberry Pi, six servo connectors, ESP32 DevKit VIN/5V
3.3V      -> ESP32-generated rail and sensor pullups
```

The 5 V converter output may be isolated from its input. This carrier
intentionally provides a single-point `GND_LINK` footprint so its output
negative can be joined to the motor and ESP32 signal ground after checking the
module datasheet. Do not assume an isolated output is already common. The
current placement sample does not yet include reverse-polarity protection;
add and verify that stage before connecting a battery.

The sample assumes a 6S Li-ion battery marketed as 24 V / 9 Ah. The motor rail
is not regulated: expect approximately 18-25.2 V over the discharge cycle.
The JGA25-2430-CE is specified as a 24 V motor, but the manufacturer does not
publish a maximum input voltage, so verify full-charge operation and motor
temperature before production. A 24 V buck-boost regulator can be added later
if constant speed or a guaranteed 24 V rail is required.

## ESP32-WROOM-32 DevKit pin map

The board uses a socket for the common 30-pin ESP32-WROOM-32 DevKit form
factor. The mapping below assumes the usual DevKit V1 header order with the
USB connector at the marked edge. Confirm the mechanical orientation against
the exact board before fabrication.

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
| Servo 5 PWM | 22 | `S5_PWM` |
| Servo 6 PWM | 32 | `S6_PWM` |
| I2C SDA | 5 | `I2C_SDA` |
| I2C SCL | 33 | `I2C_SCL` |

GPIO34-39 are input-only and are reserved for encoder signals. GPIO6-11 are
not used because they are connected to the ESP32 flash. GPIO5 is a strapping
pin; the I2C pull-up must remain high during reset.

The carrier feeds the ESP32 socket with +5 V on VIN and uses the DevKit's
regulated 3.3 V output for encoder pull-ups. The Pi's 3.3 V pin is left
unconnected, avoiding a direct tie between the Pi and ESP32 regulator outputs.
The optional I2C header includes 4.7 k pull-ups to this same ESP32 3.3 V rail.

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

### Servo connectors J20-J25

```text
1  +5V_SERVO
2  GND
3  PWM     (ESP32 3.3 V logic, 50 Hz)
```

The six servos share the 5 V converter output. The final servo model must be
known before selecting the servo fuse and connector current rating.

## Required next revision

- Add the full KiCad schematic and assign verified footprints.
- Verify the purchased 5 V converter pinout and thermal derating.
- Route high-current input, motor, and servo copper.
- Add the exact Pi 4B mounting pattern and USB-C power connection.
- Add 470-1000 uF servo bulk capacitance and 470 uF Pi bulk capacitance.
- Verify the four fitted 10 k encoder pull-ups and add input protection if
  cable length or the final encoder waveform requires it.
- Select the input fuse for the 6S battery, motor stall current, and 5 V
  converter input current; 15 A remains provisional until measured.
- Add 220 ohm series resistors on servo PWM signals.
- Add a hardware motor/servo power-enable or emergency-stop circuit.
