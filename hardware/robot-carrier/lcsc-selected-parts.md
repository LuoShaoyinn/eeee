# LCSC selected parts

This is the first concrete sourcing pass for the 80 mm x 65 mm placement trial.
All LCSC stock and assembly eligibility must be rechecked at order time.

| Ref | LCSC part | Selected part | Physical basis |
| --- | --- | --- | --- |
| D1 | C78419 | Brightking SMBJ33A | SMB / DO-214AA, 33 V reverse standoff |
| C1 | C44705 | Chengx GR227V050G13RR0VL4FP0 | 220 uF, 50 V, radial D10, 5 mm lead pitch |
| C2 | C46551024 | KNSCHA DUK035M477G20CS1AA | 470 uF, 35 V, radial D8, 3.5 mm lead pitch |
| C3 | C79119 | Panasonic EEUFR1A102L | 1000 uF, 10 V, radial D8, 3.5 mm lead pitch |
| C4 | C502153 | IHHEC C0805X104K050T | 100 nF, 50 V, X7R, 0805 |
| R1-R4, R7-R8 | C17414 | RC0805FR-0710KL | 10 kOhm, 1%, 0805 |
| R5-R6 | C55174750 | FRR0805F4701TS | 4.7 kOhm, 1%, 0805 |
| J2, J3 | C500021 | DORABO DB128L-5.08-2P-GY-S | 2-position 5.08 mm screw terminal, 16 A |
| J41 | C430602 | DORABO DB128L-5.08-3P-BK-S | 3-position 5.08 mm screw terminal, 16 A |
| J10-J13 | C7434483 | GREENCONN GPHA101-0502A001A1BA | 1x5 2.54 mm male header, rated 3 A/contact |

## Not selected yet

- `F1`: a 15 A battery fuse needs a confirmed motor-stall and converter-input
  current, then a matching holder or automotive blade-fuse footprint.
- `J1`: choose a DC jack only after confirming its current rating and the plug
  style. A typical 5.5 mm barrel jack is often unsuitable for a 15 A battery
  path.
- Servo, Pi/UART, L298N logic, and ESP32 socket headers are hand-installed
  connector hardware; select their exact body height and mating direction with
  the harness before ordering.

Primary LCSC references: [SMBJ33A C78419](https://item.szlcsc.com/79554.html),
[C1 C44705](https://item.szlcsc.com/45707.html),
[C2 C46551024](https://item.szlcsc.com/48681800.html),
[C3 C79119](https://item.szlcsc.com/80256.html),
[J2/J3 C500021](https://item.szlcsc.com/512133.html), and
[J41 C430602](https://item.szlcsc.com/426247.html).
