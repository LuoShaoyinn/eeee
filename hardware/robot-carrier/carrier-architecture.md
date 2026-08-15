# Carrier architecture

## Power tree

```text
6S Li-ion battery protected output, about 18-25.2 V
  -> J1 16 A screw terminal -> VIN_PROTECTED
                                  |-> D1 1.5KE33A TVS -> GND
     -> four JGA25-2430-CE motor power connectors, direct battery rail
     -> J2 screw terminal -> external 24 V to 5 V converter input

VIN_PROTECTED -> four JGA25-2430-CE motor power connectors
+5V       <- J3 screw terminal <- external provisional 10 A fuse
                                <- external 24 V to 5 V converter output
          -> remote Raspberry Pi harness, four servo connectors, ESP32 DevKit VIN/5V
          -> J41 screw terminal -> external L298N power terminals
3.3V      <- ESP32 DevKit 3V3 output; encoder and I2C pullups, IMU VCC
```

The 5 V converter is external and connects through two 5.08 mm screw-terminal
blocks: `J2` carries its 24 V input and `J3` receives its 5 V output. The
ESP32 DevKit's on-board regulator creates the low-current 3.3 V rail used by
the IMU, encoder pullups, and I2C pullups. Do not use that 3.3 V output for
motor or servo power.
The normal battery connection assumes its output already includes correctly
rated protection. J1 therefore connects directly to `VIN_PROTECTED`; there is
no battery fuse or fuse-link footprint in this revision. A future fuse would be
inserted in the positive lead ahead of J1.
D1 is an axial 1.5KE33A TVS across `VIN_PROTECTED` and `GND`, physically beside
J1, with its cathode toward battery positive. This is the only on-board input
protection. The current placement deliberately has no eFuse or reverse-polarity
stage, so correct connector polarity and the battery's own over-current
protection are assumptions, not functions supplied by this PCB.

The supplied IMU is a WitMotion WT901/WT901S-style 9-axis module. Its
hardware I2C supports up to 400 kHz, uses open-drain SDA/SCL, and defaults to
the 7-bit address `0x50`. Power it at 3.3 V on this carrier so the external
4.7 k pull-ups are always at safe ESP32 logic level.

The sample assumes a 6S Li-ion battery marketed as 24 V / 9 Ah. The external
24 V to 5 V converter is rated for 20 A maximum as redundant capacity, while
the expected total 5 V operating load is below 10 A. The motor rail
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
| L298N ENA / PWM | 22 | `GA25_ENA_PWM_3V3` |
| L298N IN1 | 32 | `GA25_IN1_3V3` |
| L298N IN2 | 12 | `GA25_IN2_3V3` |
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

C5 connects one shared 100 nF / 50 V bypass from `VIN_PROTECTED` to `GND` for
the nearby J10-J13 group. It suppresses high-frequency rail noise from the
integrated controllers; C1 is the shared battery-rail bulk capacitance.

PWM is active-low: GPIO high is the stopped state and GPIO low commands the
motor to run. R9-R12 provide 10 k pull-ups from M1-M4 PWM to the ESP32's 3.3 V
rail, establishing the stopped state while the GPIOs are high-impedance during
reset. For long motor cables, a 220-1000 ohm series resistor may be added inline
in a later revision. Never connect the white, orange, or yellow signal wires to
the 24 V rail.

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

For USB-C power, use JPI1 pins 1-4 as the input to a short, 3 A-rated captive
power harness ending in a male USB-C plug, rather than connecting those pins to
the Pi GPIO header. The plug/harness must implement the USB-C source-side CC
advertisement for 3 A; a two-wire VBUS/GND breakout is not sufficient evidence
of correct USB-C behavior. JPI1 pins 5-6 can remain a separate UART connection
to the Pi GPIO header. Never feed the Pi through its USB-C input and its 5 V
GPIO pins at the same time.

A rigid PCB-mounted male USB-C plug is not recommended because it transfers
cable and chassis loads directly into the Pi receptacle. If a USB-C receptacle
is fitted to the carrier instead, a standards-compliant source implementation
also requires CC attach detection and VBUS switching; two pull-up resistors on
an always-powered receptacle are only a prototype shortcut. No higher-voltage
USB-PD mode is needed for a Raspberry Pi 4: provide a low-drop 5 V/3 A path and
verify the voltage at the Pi plug under peak load.

### External buck terminals J2/J3

```text
J2 pin 1  VIN_PROTECTED -> buck IN+
J2 pin 2  GND           -> buck IN-
J3 pin 1  +5V           <- provisional 10 A fuse <- buck OUT+
J3 pin 2  GND           <- buck OUT-
```

### WT901 IMU connector J30

```text
1  +3V3    -> WT901 VCC
2  GND     -> WT901 GND
3  I2C_SDA -> WT901 SDA
4  I2C_SCL -> WT901 SCL
```

The WT901 offers UART as an alternative interface, but the carrier uses its
I2C pins. R5 and R6 provide the required 4.7 k pull-ups to 3.3 V.

### GA25-370 / L298N interface

The selected auxiliary motor is the 6 V GA25-370. It is expected to operate
under low mechanical load and is deliberately supplied from 5 V through the
L298N. Because the L298N drops voltage in its output stage, validate the actual
motor-terminal voltage, startup current, speed, and module temperature under
the intended load.

```text
J40 (three-pin Dupont logic header)
1  GA25_ENA_PWM_3V3 -> L298N ENA
2  GA25_IN1_3V3     -> L298N IN1
3  GA25_IN2_3V3     -> L298N IN2

J41 (three-position screw terminal)
1  +5V -> L298N motor-power terminal
2  GND -> L298N GND
3  +5V -> L298N 5V logic terminal
```

J40 uses a male Dupont header so it connects to the L298N's logic header with
female-to-female Dupont wires. J41 connects by screw-terminal wires to the
L298N's power terminal block. Connect the GA25-370 to L298N output A. Remove
the module's channel-A `ENA` jumper so J40 can drive ENA. Also remove the
`5V_EN` jumper and feed its 5 V logic terminal from J41 pin 3.
The module is physically separate but not electrically isolated: its ground
must join carrier ground. The provided module is specified for 2 A maximum per
bridge, so verify the exact GA25-370 startup/stall current and module temperature
before relying on it.

C4 connects 100 nF / 50 V between +5 V and ground beside J41. In addition,
solder one separate non-polarized 100 nF ceramic capacitor directly across the
GA25-370 brushed-motor terminals with very short leads. That part is attached
to the motor and is not a PCB designator. Never use a polarized capacitor
across the L298N motor output because direction changes reverse the output
polarity.

The GA25-370 encoder is intentionally unused. Insulate its encoder wires and
leave them disconnected. The four `M1_ENC`-`M4_ENC` inputs remain reserved for
the four JGA25-2430-CE motors. The remaining unallocated exposed pins are
GPIO0, GPIO2, and GPIO15, all of which are boot-strapping pins. This revision
therefore has no unused safe GPIO pair for the GA25-370 quadrature outputs.

## Required next revision

- Keep the KiCad schematic and PCB net assignments synchronized after routing changes.
- Repack the current 80 mm x 65 mm placement trial into the requested 60 mm x
  70 mm production outline while retaining tool access around the hand-soldered
  parts.
- Verify the purchased 5 V converter pinout and thermal derating.
- Route high-current input, motor, and servo copper.
- Keep D1, C4, and C5 close to the connectors they protect or bypass when finalizing placement.
- Select the final Raspberry Pi USB-C power cable/connector and keep JPI1 only
  for UART if USB-C power replaces GPIO-header power.
- Measure the 5 V rail at J3, the servos, and the Pi under peak load before
  deciding whether the shared C3 bulk capacitor needs additional local bulk.
- Test the external L298N at the exact GA25-370 startup/stall current and replace it
  if voltage drop or temperature is excessive.
- Verify the four fitted 10 k encoder pull-ups and add input protection if
  cable length or the final encoder waveform requires it.
- Verify the assumed battery protection current and interruption ratings
  against motor stall current and the 5 V converter input current.
- Add 220 ohm series resistors on servo PWM and L298N logic signals.
- Verify R9-R12 hold all four active-low JGA25-2430-CE PWM inputs high during
  simultaneous power-up and ESP32 reset.
- Add 5 V overcurrent protection sized for the verified sub-10 A load so the
  20 A converter cannot overload J3 or the PCB during a downstream fault.
- Add a hardware motor/servo power-enable or emergency-stop circuit.
