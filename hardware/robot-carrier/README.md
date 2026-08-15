# ESP32-WROOM-32 / Raspberry Pi 4B carrier board

This directory contains the first KiCad carrier-board sample for the mobile
robot. It is a placement and interface prototype, not yet a manufacturing
release. Power routing, thermal validation, and exact mechanical dimensions
must be reviewed before ordering a PCB.

## Project files

- `robot-carrier.kicad_pcb`: current 80 mm x 65 mm two-sided placement trial.
  The requested production envelope is 60 mm x 70 mm; that repacking has not
  yet been applied and must be completed before routing or ordering.
- `robot-carrier.kicad_sch`: authoritative pre-routing schematic.
- `robot-carrier.kicad_pro`: KiCad project metadata.
- `RobotCarrier.kicad_sym`, `Custom.pretty/`, `sym-lib-table`, and
  `fp-lib-table`: project-local symbols and footprints.
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

The current JPI1 power pins may instead feed a short, 3 A-rated captive cable
ending in a USB-C male plug, so the Pi receives power through its USB-C input
rather than its GPIO header. Use a cable/plug assembly that correctly advertises
3 A on the USB-C CC connection; do not use an unverified VBUS/GND-only breakout.
UART remains on JPI1 pins 5-6. Do not power the Pi through USB-C and its 5 V GPIO
pins simultaneously.

## External power converter

The 24 V to 5 V converter is off-board. The selected converter is rated for
20 A maximum output to provide headroom; the expected total 5 V operating load
is below 10 A. Its input rating must include the 6S battery's 25.2 V
full-charge voltage. `J2` is its 24 V input terminal and `J3` is its 5 V output
terminal. The converter's 20 A capability does not mean the carrier normally
draws 20 A. Fit a provisional 10 A fuse or equivalent current-limited output
between the converter and J3; the protection setting must remain below both
J3's 16 A rating and the completed PCB copper rating.

The 6S Li-ion battery is used directly for the motor rail. Its advertised
24 V rating is nominal; the actual rail is approximately 18-25.2 V. The
previous 24 V stabilizer was removed to simplify the design. Add a 24 V
buck-boost stage only if the final motor test requires a constant voltage.
The default input wiring assumes the battery pack already includes correctly
rated protection. J1 connects directly to `VIN_PROTECTED`; this revision has
no battery fuse, eFuse, fuse-link footprint, or reverse-polarity stage. D1 is
the only on-board input protection: an axial 1.5KE33A TVS connected across the
battery rail beside J1. Verify battery polarity before connection.

The tested JGA25-2430-CE controller accepts direct 3.3 V PWM and direction
signals from the ESP32. The carrier does not use the previously considered
74AHCT125 level shifters. The motor connectors carry direct battery power;
the `*_3V3` names refer only to logic signals, never to motor power. Motor
signal traces share the carrier ground with the motor power return; the yellow
feedback input has a 10 k pull-up to the ESP32 DevKit's 3.3 V output. R9-R12
also pull the four active-low PWM inputs up to 3.3 V so an ESP32 reset produces
the stopped command rather than leaving the inputs floating.

The selected auxiliary motor is the low-load 6 V GA25-370. It is intentionally
operated from the 5 V rail through the physically external L298N module, so its
available terminal voltage, speed, and torque will be lower than when driven
directly at its nominal 6 V. Verify motor-terminal voltage, startup current,
loaded speed, and L298N temperature on the assembled system.

`J40` is a three-pin Dupont logic header for `ENA`, `IN1`, and `IN2`; `J41` is
a three-position screw terminal for motor 5 V, ground, and logic 5 V. The motor
connects to the L298N module's own output screw terminals. The module is not
galvanically isolated: its ground must join carrier ground. Remove both the
channel-A `ENA` jumper and the `5V_EN` jumper. Drive `ENA` from J40 pin 1 and
feed the module's 5 V logic terminal from J41 pin 3.

C5 provides one shared 100 nF bypass from the direct battery rail to ground for
the four nearby JGA motor connectors. C4 provides one shared 100 nF bypass from
+5 V to ground beside J41. These board capacitors shorten high-frequency
supply-current loops but do not replace C1/C3 bulk capacitance or the
battery-side TVS. For the brushed
GA25-370 itself, solder a separate non-polarized 100 nF ceramic capacitor
directly across the two motor terminals with the leads as short as possible.
Do not place a polarized capacitor across the L298N output terminals because
their polarity reverses with motor direction.

The GA25-370 encoder is not used. Its feedback wires remain insulated and
disconnected. All four M1-M4 encoder inputs are required and remain dedicated
to the four JGA25-2430-CE motors. The only otherwise unused exposed GPIOs are
GPIO0, GPIO2, and GPIO15, all of which are boot-strapping pins; there is no
unused safe GPIO pair for the GA25-370 quadrature encoder.

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

1. Confirm the assumed battery protection is suitable for the wiring,
   motor-stall current, and converter input. Populate D1 beside J1 with its
   cathode on `VIN_PROTECTED` and anode on `GND`. This revision intentionally
   has no reverse-polarity stage, so verify J1 polarity before every battery
   connection.
2. Wire the external buck as `J2 -> IN` and `OUT -> provisional 10 A fuse or
   equivalent current limit -> J3`, only after confirming its pinout against
   the purchased manufacturer's part.
3. Power the board without motors or servos and verify the battery rail and
   `+5V` at J3 and at the remote Pi harness.
4. The carrier and all attached devices are powered on together. The L298N
   channel remains disabled during ESP32 startup through R8's hardware `ENA`
   pull-down. R9-R12 hold the four active-low JGA25-2430-CE PWM inputs high
   while the ESP32 GPIOs are undriven during reset. These pull-ups do not stop
   an MCPWM peripheral that continues its last waveform during a CPU hang; use
   an independent watchdog or motor-power enable if that failure must stop the
   motors.
5. Add a hardware emergency-stop that removes motor and servo power.
6. Verify the Pi UART harness direction: ESP32 TX goes to Pi RX, and Pi TX
   goes to ESP32 RX. The UART uses the ESP32's UART0 pins and must be isolated
   or disconnected while using the DevKit USB port for flashing if contention
   occurs.

## KiCad checks

Run these commands from this directory:

```bash
kicad-cli sch erc robot-carrier.kicad_sch \
  --output erc-report.txt --severity-all
kicad-cli pcb drc robot-carrier.kicad_pcb \
  --output drc-report.txt
kicad-cli pcb render robot-carrier.kicad_pcb \
  --output robot-carrier-3d.png
kicad-cli pcb export gerbers robot-carrier.kicad_pcb \
  --output gerbers
```

The schematic is the authoritative electrical source and currently passes
KiCad CLI ERC. The PCB is deliberately not routed. The next design pass must
repack the placement into the requested 60 mm x 70 mm outline without reducing
the hand-solder clearances, update the PCB from the schematic, then route the
high-current nets with widths and copper weight justified by the measured
current and allowed temperature rise.

This revision targets full hand assembly. R1-R12 are 1206 parts on custom
extended pads (1.6 mm x 2.0 mm per pad), leaving exposed copper beyond both
ends for an iron tip. C4 and C5 are through-hole ceramics on 5.08 mm pitch with
2.4 mm pads. There are no small SMD capacitors to install. JLC/LCSC Basic
versus Extended assembly classification is therefore not a selection
constraint, although stock must still be rechecked before ordering.

For easiest assembly, solder R1-R12 before installing the tall connectors and
electrolytic capacitors. Tin one resistor pad, reflow that end while positioning
the part with tweezers, then solder the other end. C4 and C5 are non-polarized;
C1 and C3 are polarized and must follow the `+` marking. Install D1 with its
cathode toward `VIN_PROTECTED`. The PCB spacing is intended to leave access to
the resistor ends and to the underside of all through-hole pads.

## Reference material versus selected parts

[实验材料说明.pdf](../../docs/实验材料说明.pdf) is the provided catalogue of
available example parts, not the carrier's authoritative BOM. In particular,
its 12 V motor example and LM2596 supply example are not the selected auxiliary
motor or 5 V converter. This carrier uses the 6 V GA25-370 at low load, the
pictured L298N module, and an external 24 V to 5 V converter rated for 20 A
maximum output. The Markdown files in this directory and `lcsc-bom.csv` define
the current design choices.

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
kicad-cli pcb export ipcd356 \
  robot-carrier.kicad_pcb --output /tmp/robot-carrier.ipc356
```

Before routing, run schematic ERC and use KiCad's **Update PCB from Schematic**
workflow. Re-run PCB DRC after every placement or routing pass; unrouted-net
reports are expected until routing is complete, but shorts and clearance
violations are not acceptable.

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
