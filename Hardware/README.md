# Hardware — strobed wand for extrinsic calibration

KiCad 10 schematics for the pulsed marker wand that feeds `OpenMV/ExtrinsicsCalib`.
Two boards, schematic-only so far (no PCB layout yet).

| Directory | Build | Parts | Does |
|---|---|---|---|
| `WandStrobeDriver/` | ×1, at the handle | 20 | TLC555 astable + high-side P-FET; strobes all four markers together |
| `WandMarker/` | ×4, one per marker | 8 | 4× Kingbright L-7113SF7C in series + one ballast resistor |

```bash
python3 gen_schematics.py
```

`gen_schematics.py` writes both `.kicad_sch` files; change a value there and re-run
rather than editing the schematic by hand. `kisch.py` is a minimal schematic writer
and `kisym.py` pulls symbol geometry out of the stock KiCad libraries, so wire
endpoints come from real pin positions instead of being typed in.

## Rev B — what changed and why

Rev A ran **12 V** and put a **constant-current sink** on every marker board.
Both are gone.

The sink cost 1.6 V of headroom — a Vbe of sense, a Vce of regulation, and the
input Schottky — and that 1.6 V was the *entire* reason rev A needed 12 V. What it
bought was marker-to-marker matching of roughly 5 %. The bench data already shows
σ moving only ~3 % across a 20 % exposure range, and run-to-run repeatability of
the characterisation itself is ~3 %. So the sink was buying matching finer than the
measurement can resolve, at the price of a higher supply and eight extra parts per
marker board.

**The marker board is now the board that was already characterised, plus a
connector.** That is also why the existing boards plug straight into the driver.

Rev A is in git history if supply sensitivity ever turns out to matter.

## The rig

```
D
|
|                 D arm perpendicular at A, length >= |AC|/3
A-------------B---------C
<-----2/3-----><--1/3--->
```

A = origin, C = +X, D = +Y, as `ExtrinsicsCalib/README.md` defines them. Two wires
run from the driver to each marker: switched +9 V and a common ground. One timer
drives all four, so "dark" always means occluded, never out of phase.

## Why there is no diode across R2

This is the one place where the obvious simplification is the wrong one.

The standard trick for sub-50 % duty on a 555 is a diode across R2, and it assumes
the LEDs are on when **OUT is high**. They are not. `Q1` is P-channel on the
**high side** and conducts when **OUT is low** — so the short state is already the
state we want, and a plain astable with `R1 >> R2` gives 0.36 % duty with no diode
at all.

Adding the diode would force `R1` small, and `R1` then sinks `Vcc/R1` through the
DISCH pin for the whole 327 ms off-time — about **20 mA**, on a battery, against
the **9 µA** the circuit actually draws as built.

High-side switching also means every marker's return is permanently grounded, so a
chafed wire against the frame is a non-event rather than a short.

## Timing — three fixed parts

```
t_on   = 0.693 · R2 · C6              1075 µs
t_off  = 0.693 · (R1+R2) · C6          327 ms      ->  3.05 Hz
```

`R1 = 1 M`, `R2 = 3k3`, `C6 = 470 nF`. 1 MΩ is legal **only** because the TLC555 is
CMOS (pA input current); a bipolar NE555 would not hold this operating point.

3 Hz is `buildEasyWand`'s default `PulseHz`. Pulse grouping needs frame period <
pulse period with margin, and the 555 is a free-running RC oscillator — expect a few
percent of error and drift. That is fine, because alignment fits an offset and rate
per camera. Do not treat the strobe period as a known constant.

**`R2` sets pulse width, which is the brightness control.** Peak current is fixed by
the ballast resistor on each marker board, so the light delivered into one exposure
is `I × t_on`. Fit `R2` through-hole and clip a 5 k pot across its pads for bring-up,
then solder the nearest E24 value. The rate barely moves with it:

| R2 | t_on | | R2 | t_on |
|---|---|---|---|---|
| 1k8 | 586 µs | | **3k3** | **1075 µs** (fitted) |
| 2k2 | 717 µs | | 3k9 | 1270 µs |
| 2k7 | 879 µs | | 4k7 | 1531 µs |

**Every resistor on both boards is an E12 value** — 1M, 3k3, 220, 100k, 22k, 100,
and 68R on the markers. Nothing needs series pairs or an E24 stock.

The exact number does not matter. The 470 nF film capacitor is ±5 % on its own,
which is ±54 µs — larger than the gap between adjacent E12 steps at this end of
the range. `R2` is set empirically against a camera reading anyway, so treat
whatever you fit as a starting point.

## Supply — 9 V is sufficient

| Load | Average |
|---|---|
| TLC555 | ~1 mA |
| Timing network | 9 µA |
| D2 power indicator | ~0.35 mA |
| LEDs, 4 × 36 mA × 0.36 % | 0.52 mA |
| **Total** | **~1.9 mA** |

6×AA alkaline, or a PP3 for bench work. The timer, not the LEDs, dominates.

Headroom at 9 V, after the input Schottky (0.45 V) and bulk-cap droop (~0.2 V),
leaves 8.35 V across four LEDs and the ballast resistor. Comfortable — the string is
about 5.9 V, not the 6.4 V a flat 1.6 V per LED would suggest (see below).

`C1 = 1000 µF` covers the pulse: 4 × 36 mA × 1173 µs = 169 µC → 0.17 V droop.

## The emitter

**Kingbright L-7113SF7C**, 5 mm through-hole, GaAlAs, 850 nm (datasheet DSAB9772 V.16B).

| Parameter | Value | Conditions |
|---|---|---|
| Forward voltage | 1.4 typ / **1.6 max** | **at I_F = 20 mA** |
| DC forward current | **50 mA** | absolute max |
| Peak forward current | 1 A | 1/100 duty, **10 µs** pulse width |
| Power dissipation | 80 mW | absolute max |
| Radiant intensity | 30 typ mW/sr @ 20 mA, 90 @ 50 mA | |
| Viewing angle 2θ½ | **20°** | narrow — this is the ring artefact |

**1.6 V is not "the" forward drop.** It is the maximum *at 20 mA*. Vf climbs with
current and falls below 1.6 V under 20 mA. At 36 mA expect about 1.48 V each, so
four in series is ~5.9 V.

At 36 mA and 0.36 % duty each LED averages **0.19 mW against an 80 mW limit**.

## Setting the LED current

`R1` on each marker board. Same value on all four. Computed at 9 V less the driver's
Schottky and bulk-cap droop, using the real Vf curve rather than a flat 1.6 V:

| R1 | I_LED | Pulse for peak 210 at 4 m | Smear @ 0.3 m/s |
|---|---|---|---|
| 33R | 61 mA | 739 µs | 0.29 px |
| 47R | 48 mA | 951 µs | 0.37 px |
| **68R** | **36 mA** (fitted, matches the bench build) | **1263 µs** | **0.49 px** |
| 100R | 26 mA | 1729 µs | 0.67 px |

Measure across `TP1`–`TP2` with a scope during a pulse: `I = V(TP1) / R1`.

### The price of dropping the constant-current sink

0.3 V of battery sag moves the current **9 %**. The peak pixel value moves with it,
so re-target the exposure occasionally over a long session. It is a slow drift, not
noise, and the bench data shows σ changing only ~3 % across a 20 % exposure range —
so it costs well under 1 % of centroid accuracy. That is the whole trade, stated
plainly.

### Check the workbook number

`LED_characterisation.xlsx` lists **38.2 mA** for this build, computed by
`IR_LED_Parts_Calculator` from an assumed **Vf = 1.6 V** — which is the datasheet
*maximum at 20 mA*, not the value at 38 mA. With the real curve, 9 V through 68 Ω
gives about **43 mA**. Worth measuring once and correcting in the sheet, since that
column appears in the thesis.

## Bench test: build the driver first

Nothing in the driver cares about 9 V versus 12 V — the TLC555 runs 2–15 V and the
P-FET gate swings whatever the rail is. So build the driver, feed it a bench supply,
and hang the **existing** characterisation markers off J2–J5.

| Rail | Existing 68 Ω board draws |
|---|---|
| **9 V** | **36 mA** — the characterised operating point |
| 12 V | 74 mA |

At 9 V the strobe test is directly comparable with the DC rows in `A_Bench`. The
equivalence worth confirming is that the *peak pixel value* matches the DC bench row
once `R2` is set, because the camera in Regime B sees the peak, not the average.

Probe `TP1` (gate) against `TP3` (GND) on a scope before connecting any LED — that
confirms period and pulse width on their own.

## Two parts you must not substitute casually

**Q1 (driver).** `DMP3098L-7` — **P-channel enhancement mode**, SOT-23
(Diodes Inc DS31447). The ratings that matter here:

| Parameter | Value | Why it matters |
|---|---|---|
| V_GSS | **±20 V** | the gate swings the whole rail, so this must exceed 9 V |
| V_GS(th) | −1.0 / −1.8 / −2.1 V | at −9 V the part is hard on, far past threshold |
| R_DS(on) | 70 mΩ max @ V_GS = −10 V | 10 mV across it at the 144 mA pulse |
| V_DSS | −30 V | 3× the rail |
| I_D | −3.8 A cont., −11 A pulsed | wildly over-specified at 144 mA, which is fine |
| Q_g | 7.8 nC @ −10 V | ~190 ns edge through R3, against a 1173 µs pulse |

**It must be enhancement mode, not depletion.** An enhancement device is OFF at
V_GS = 0, which is what R4 holds it at whenever the timer is not driving — so a
missing or unpowered TLC555 leaves the markers dark. A depletion device is ON at
V_GS = 0 and would sit the strings at DC with no strobe at all.

Any substitute needs V_GSS comfortably above 9 V. Do not fit a ±8 V part such as
Si2301; check the rating rather than assuming, since many SOT-23 P-FETs are ±12 V
and some are ±8 V.

**D1–D4 (marker).** Kingbright `L-7113SF7C`, footprint `LED_THT:LED_D5.0mm`. They are
marked **DNP** so a turnkey assembler skips them — they are hand-soldered so the
standoff to the diffuser can be set by hand. DNP excludes a part from the BOM and
position files, **not from the PCB**: the pads are still required.

## PCB layout - driver board

`gen_pcb.py` writes `WandStrobeDriver.kicad_pcb`. Nets are read from the
schematic netlist, so the board cannot silently disagree with the schematic
about connectivity; DRC is the check that the routing implements them.

```bash
kicad-cli sch export netlist --format kicadsexpr \
    -o WandStrobeDriver/WandStrobeDriver.net WandStrobeDriver/WandStrobeDriver.kicad_sch
python3 gen_pcb.py          # writes the board, then runs DRC
```

**80 x 60 mm, two layers, 1 oz.** 26 footprints, 95 track segments, 7 vias.
**DRC: 0 violations, 0 unconnected pads.**

The layout follows the rule from the design notes - switching current and the
high-impedance timing network occupy separate regions:

| Region | Holds |
|---|---|
| Top edge | J1, SW1, F1, D1 - the input chain, left to right |
| Upper middle | C1 bulk, C2, the indicator, R10/C3/C4 filtering VCC |
| Right | Q1 and the four marker connectors, so the pulse loop stays short |
| Lower left | U1 with R1, R2, C6, C5 - nothing switched crosses it |

**B.Cu is the ground return.** Only two signals use the back layer: the gate
escape from U1 pin 3 along `y = 83` (x 117.8 to 150) and its test-point stub at
`x = 142`. Every GND crossing of that line is placed outside that span, at
`x = 102.5` or `x = 155`, so the return is one connected tree. Through-hole
grounds reach it directly; the five SMD grounds each get a via.

There is also a B.Cu GND zone. It is **not filled in the file** - `kicad-cli`
cannot fill zones - but nothing depends on it: GND is routed with real tracks,
which is why DRC reports zero unconnected pads. Open the board in KiCad and
`Edit > Fill All Zones` to add the pour before making gerbers.

Fabrication outputs are in `WandStrobeDriver/fab/` (gerbers + Excellon drill),
and `WandStrobeDriver-PCB.pdf` is the top-side plot.

### If you move anything

Two clearances are deliberate and tight enough to check after any edit:

- The DISCH track at `y = 92.54` passes R2 pad 2 with 0.46 mm.
- The `+9V` feed to Q1 source at `x = 149` passes the gate via at (150, 83)
  with 0.4 mm.

## Still open

- **Marker board layout.** Not laid out yet. On the marker board the LED ring
  The LED ring must be exactly concentric with the centre datum (an asymmetric
  ring offsets the blob centroid by a fixed bias no filter can see), and the emitters aimed
  **tangentially** — the 20° viewing angle is what makes the marker ring, not the
  shell diameter. Do not fix the ring radius until Test G reports, because radius and
  wall thickness move the LED-to-wall standoff together.
- **Ambient IR.** Regime B needs a ~28 ms exposure and Test F (`F_Ambient`) has not
  been run. If the background floods it, the fallback is Regime A — short exposure,
  long pulse — which this board reaches by fitting a larger `R2`.
- **Which side D goes on.** The frame defines the *calibration* XY plane, not an
  electrical ground, so every marker keeps its own ground wire. easyWand takes
  origin, +X and +Y and lets **Z follow the right-hand rule**, so `A→C × A→D` must
  point **up**. Lay the rig flat, stand at A looking toward C, and D must be on your
  **left**. Put D on the right and Z points into the floor: every camera then solves
  below ground, mirrored, and it will not look like a sign error — it will look like
  a bad calibration.

## Verification

Both schematics pass with zero ERC violations, and the netlists have been checked
against the intended circuit:

```bash
kicad-cli sch erc --severity-all -o /tmp/erc.rpt WandStrobeDriver/WandStrobeDriver.kicad_sch
kicad-cli sch export netlist --format kicadsexpr -o /tmp/n.net WandMarker/WandMarker.kicad_sch
```
