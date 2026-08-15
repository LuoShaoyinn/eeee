# LCSC selected parts

This is the first concrete sourcing pass for the implemented 60 mm x 73.5 mm
placement. All LCSC stock and assembly eligibility must be rechecked at order
time.

| Ref | LCSC part | Selected part | Physical basis | Assembly plan |
| --- | --- | --- | --- | --- |
| D1 | C18198472 | DOWO 1.5KE33A | DO-201AD axial, 28.2 V reverse standoff, unidirectional | Through-hole, hand install beside J1 |
| C1 | C51953331 | Econd PH471M063G200B000 | 470 uF, 63 V, polymer radial D10, 5 mm lead pitch | Through-hole, hand install |
| C3 | C79119 | Panasonic EEUFR1A102L | 1000 uF, 10 V, radial D8, 3.5 mm lead pitch | Through-hole, hand install |
| C4-C5 | C84772 | FH CT4-0805B104K500F3 | 100 nF, 50 V, X7R, 5.08 mm lead pitch | Through-hole, hand install |
| R1-R4, R7-R12 | C17902 | UNI-ROYAL 1206W4F1002T5E | 10 kOhm, 1%, 0.25 W, 1206 | Hand solder |
| J1-J3 | C500021 | DORABO DB128L-5.08-2P-GY-S | 2-position 5.08 mm screw terminal, 16 A | Through-hole, hand install |
| J41 | C430602 | DORABO DB128L-5.08-3P-BK-S | 3-position 5.08 mm screw terminal, 16 A | Through-hole, hand install |
| J10-J13 | C7434483 | GREENCONN GPHA101-0502A001A1BA | 1x5 2.54 mm male header, rated 3 A/contact | Through-hole, hand install |

The external 24 V to 5 V converter is rated for 20 A maximum to provide
headroom, but the expected operating load is below 10 A. J3 and the routed 5 V
copper must be protected so a downstream fault cannot use the converter's full
20 A capability. The external auxiliary motor is the 6 V GA25-370 operated at
low load through the provided L298N module; its encoder is intentionally not
connected.

## Hand-assembly strategy and external protection

- This revision assumes a bare fabricated PCB with every component installed
  by hand. R1-R4 and R7-R12 remain 1206 parts but use custom 1.6 mm x 2.0 mm
  solder pads, which expose more copper beyond each resistor end. C4-C5 are leaded parts
  with 5.08 mm spacing and 2.4 mm through-hole pads; there are no small SMD
  capacitors left to install.
- D1 is an axial 1.5KE33A so it is easy to install by hand. It is placed across
  `VIN_PROTECTED` and `GND` beside J1; install its cathode toward battery
  positive. It limits transients but does not provide reverse-polarity or
  over-current protection.
- C5 is one shared 100 nF bypass for the J10-J13 motor-power distribution. C4
  is one shared 100 nF bypass beside the L298N supply connector J41. Separately
  solder one non-polarized
  100 nF ceramic capacitor directly across the GA25-370 motor terminals with
  the leads as short as possible; that off-board capacitor is not in the PCB
  BOM.
- The battery is assumed to provide its own correctly rated protection. J1
  connects directly to `VIN_PROTECTED`; there is no fuse/link footprint.
- Fit a provisional 10 A inline fuse between the external buck output and J3.
  Increase it only after measuring the real 5 V load, and never above J3's
  16 A connector rating or the routed-copper rating.
- Servo, Pi/UART, L298N logic, and ESP32 socket headers are hand-installed
  connector hardware; select their exact body height and mating direction with
  the harness before ordering.

Primary LCSC references: [1.5KE33A C18198472](https://www.lcsc.com/product-detail/C18198472.html),
[10 kOhm 1206 C17902](https://www.lcsc.com/product-detail/C17902.html),
[100 nF through-hole C84772](https://www.lcsc.com/product-detail/C84772.html),
[C1 C51953331](https://www.lcsc.com/product-detail/C51953331.html),
[C3 C79119](https://item.szlcsc.com/80256.html),
[J1-J3 C500021](https://item.szlcsc.com/512133.html), and
[J41 C430602](https://item.szlcsc.com/426247.html).
