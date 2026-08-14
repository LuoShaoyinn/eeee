# ESP32-WROOM-32 / Raspberry Pi 4B carrier board

This directory contains the first KiCad carrier-board sample for the mobile
robot. It is a placement and interface prototype, not yet a manufacturing
release. Power routing, thermal validation, and exact mechanical dimensions
must be reviewed before ordering a PCB.

## Project files

- `robot-carrier.kicad_pcb`: 80 mm x 65 mm dense two-sided placement trial.
- `robot-carrier.kicad_pro`: KiCad project metadata.
- `carrier-architecture.md`: power tree, pin map, connector pinouts, and bring-up notes.
- `lcsc-selected-parts.md`: selected purchasable LCSC parts and remaining
  safety-critical sourcing decisions.

The board keeps the ESP32 DevKit on the carrier. The Raspberry Pi 4B is
physically separate and connects through `JPI1` using a jumper harness for
5 V, ground, and 3.3 V UART. The ESP32 socket is the common 38-pin DevKitC
header order, not a bare WROOM module footprint. It is the 38-pin, Type-C
DevKitC-style board shown by the supplied photograph, mounted on the underside
using two 1x19 female Dupont headers. Its mechanical keep-out is 27.94 mm x
54.36 mm: 25.40 mm between header rows and 45.72 mm between the end header
pads.

## External power converter

The 24 V to 5 V converter is off-board. Use a four-wire module rated for at
least 10 A continuous (15 A preferred), with an input rating that includes the
6S battery's 25.2 V full-charge voltage. `J2` is its 24 V input terminal and
`J3` is its 5 V output terminal.

The 6S Li-ion battery is used directly for the motor rail. Its advertised
24 V rating is nominal; the actual rail is approximately 18-25.2 V. The
previous 24 V stabilizer was removed to simplify the design. Add a 24 V
buck-boost stage only if the final motor test requires a constant voltage.

The tested JGA25-2430-CE controller accepts direct 3.3 V PWM and direction
signals from the ESP32. The carrier does not use the previously considered
74AHCT125 level shifters. The motor connectors carry direct battery power;
the `*_3V3` names refer only to logic signals, never to motor power. Motor
signal traces share the carrier ground with the motor power return; the yellow
feedback input has a 10 k pull-up to the ESP32 DevKit's 3.3 V output.

The GM25-370 uses a physically external L298N module. `J40` is a three-pin
Dupont logic header for `ENA`, `IN1`, and `IN2`; `J41` is a three-position
screw terminal for motor 5 V, ground, and logic 5 V. The motor connects to the
L298N module's own output screw terminals. The module is not galvanically
isolated: its ground must join carrier ground. Remove its `5V_EN` jumper and
feed the 5 V logic terminal from J41 pin 3.

The supplied WitMotion WT901/WT901S-style IMU connects at `J30` using I2C.
It supports hardware I2C up to 400 kHz and uses open-drain SDA/SCL; the
carrier's external 4.7 k pull-ups therefore remain fitted. It is powered from
the ESP32 DevKit's 3.3 V output, not 5 V, so the bus never exceeds ESP32
logic voltage. Its default I2C address is `0x50`.

The 5 V converter is specified for the battery input range. The motor rail is
not regulated, so confirm the motor's full-charge voltage tolerance and use a
hardware emergency stop for bring-up.

## Safety status

Before connecting a battery:

1. Populate and verify the input fuse and TVS. The current sample still needs
   a verified reverse-polarity stage before battery connection.
2. Wire the external buck as `J2 -> IN` and `OUT -> J3`, only after confirming
   its pinout against the purchased manufacturer's part.
3. Power the board without motors or servos and verify the battery rail and
   `+5V` at J3 and at the remote Pi harness.
4. Set the ESP32 motor PWM outputs high during reset; the motor PWM input is
   active-low, so high is the safe stopped state.
5. Add a hardware emergency-stop that removes motor and servo power.
6. Verify the Pi UART harness direction: ESP32 TX goes to Pi RX, and Pi TX
   goes to ESP32 RX. The UART uses the ESP32's UART0 pins and must be isolated
   or disconnected while using the DevKit USB port for flashing if contention
   occurs.

## KiCad checks

Run these commands from this directory:

```bash
HOME=/tmp/kicad-home kicad-cli pcb drc robot-carrier.kicad_pcb \
  --output drc-report.txt
HOME=/tmp/kicad-home kicad-cli pcb render robot-carrier.kicad_pcb \
  --output robot-carrier-3d.png
HOME=/tmp/kicad-home kicad-cli pcb export gerbers robot-carrier.kicad_pcb \
  --output gerbers
```

The first PCB file is deliberately not autorouted. The next design pass must
route the high-current power nets with measured copper widths and add the
full schematic/ERC source.

## Visual inspection and logic checks

Open the PCB in KiCad's PCB Editor:

```bash
pcbnew hardware/robot-carrier/robot-carrier.kicad_pcb
```

In PCB Editor:

1. Use the Appearance panel's **Nets** tab to show or hide a net. Right-click
   a net and choose **Highlight** to dim everything except that connection.
2. Turn on the ratsnest to see every still-unrouted connection. With this
   prototype, ratsnest lines are expected because copper routing is not yet
   present.
3. Use **Inspect -> Net Inspector** to verify the pads belonging to a net.
   For example, `M1_PWM_3V3` must contain ESP32 GPIO13 and motor connector J10
   pad 3; `M1_ENC` must contain GPIO34, J10 pad 5, and R1 pad 1.
4. Run **Inspect -> Design Rules Checker**. Missing connections are expected
   now; shorts, clearance errors, and wrong-net connections are not acceptable.
5. Use **View -> 3D Viewer** to inspect board orientation, sockets, connector
   positions, and mounting-hole clearance. It does not verify electrical
   connectivity.

For a text netlist report:

```bash
HOME=/tmp/kicad-home kicad-cli pcb export ipcd356 \
  robot-carrier.kicad_pcb --output /tmp/robot-carrier.ipc356
```

This project currently has no `.kicad_sch` file, so schematic ERC and
schematic-to-PCB parity checking are not available yet. The named PCB nets and
`carrier-architecture.md` are the current logic reference.

## LCSC / JLC production workflow

LCSC PCBA currently requests an RS-274X Gerber archive, BOM, and pick-and-place
file. It supports both SMD and THT assembly, but availability and assembly
eligibility must be checked for every selected component at order time.

The production workflow for this project is:

1. Verify the board outline, layer count, copper thickness, minimum track
   width, and connector drill sizes against the current LCSC quotation page.
2. Select exact LCSC part numbers for all SMD resistors, capacitors,
   protection parts, and connectors.
3. Keep the Pi, ESP32 DevKit, and high-power converter modules as customer-
   supplied or hand-installed parts unless LCSC confirms the exact package and
   assembly service.
4. Export KiCad Gerbers, NC drill, BOM, and CPL/pick-and-place data.
5. Upload the package to LCSC and inspect its rendered component placement
   before paying for fabrication.

The LCSC PCBA service documentation lists Gerbers, BOM, and pick-and-place as
the required PCBA inputs and supports THT assembly. See
<https://www.lcsc.com/pcba> and
<https://www.lcsc.com/faqs/pcba/pcb-pcba-files-for-quotation>.
