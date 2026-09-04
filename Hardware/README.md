# Hardware — strobed wand for extrinsic calibration

KiCad 10 schematics for the pulsed marker wand that feeds `OpenMV/ExtrinsicsCalib`.
Two boards, both schematic-only so far (no PCB layout yet).

| Directory | Build | Does |
|---|---|---|
| `WandStrobeDriver/` | ×1, at the handle | TLC555 astable + high-side P-FET; strobes all four markers together |
| `WandMarker/` | ×4, one per marker | 4× 850 nm IR LED in series + a constant-current sink |

`gen_schematics.py` writes both `.kicad_sch` files. Change a value there and re-run
rather than editing the schematic by hand — the design notes live in the same file
as the components they justify, so they cannot drift apart.

```bash
python3 gen_schematics.py
```

`kisch.py` is a minimal schematic writer and `kisym.py` pulls symbol geometry out
of the stock KiCad libraries, so wire endpoints are derived from real pin
positions instead of typed in.

## The rig

```
D
|
|                 D arm perpendicular at A, length >= |AC|/3
A-------------B---------C
<-----2/3-----><--1/3--->
```

A = origin, C = +X, D = +Y, exactly as `ExtrinsicsCalib/README.md` defines them.
Two wires run from the driver to each marker: a switched +12 V and a common
ground. One timer drives all four, so "dark" always means occluded, never
out of phase.

## Why the circuit is shaped this way

**Regime B — long camera exposure, short flash.** The flash, not the shutter,
defines the instant every camera photographs. That puts the strobe at roughly
**0.09 % duty** (300 µs every 333 ms), and that is the fact the whole design
follows from.

**The short state must be OUT *low*.** A plain 555 astable cannot put its short
state on OUT high. `NOTES.md` suggests the diode-across-R2 trick for low duty,
but that arrangement draws `Vcc/R1` through the DISCH pin for the entire 333 ms
off-time — about 20 mA, more than the LEDs use on average, on a battery. Instead
R1+RV1 ≫ R2+RV2 puts the short state on OUT **low**, and a **P-channel high-side**
FET conducts when OUT is low. Quiescent drops to ~12 µA of timing current, and
the shared node becomes ground, which is the wiring the wand wants anyway.

**1 MΩ timing resistors are only legal because the TLC555 is CMOS** (pA input
current). A bipolar NE555 would not hold this operating point.

**Constant-current sinks, not ballast resistors.** `I_LED = Vbe(Q2)/R2`,
independent of the supply, of C1 drooping during the pulse, and of Q1's beta.
All four markers therefore land on the same current without matched LEDs, so a
single camera exposure targets every marker at once — and a sagging battery dims
nothing at all until the sink drops out.

**Pulse width, not current, is the brightness knob.** Because peak current is
fixed by the sinks, the light delivered into one exposure is set by pulse width.
RV2 at the handle moves all four markers together and *cannot* unmatch them,
which a per-marker current pot would. Where `ExtrinsicsCalib/README.md` step 2
says "turn the LED trimpot", that is **RV2**.

## Timing

Plain astable, no diode:

```
t_on   = 0.693 · (R2+RV2) · C6          153 µs … 804 µs   (300 µs nominal)
t_off  = 0.693 · (R1+RV1+R2+RV2) · C6
f      = 6.5 Hz … 2.1 Hz                3 Hz at RV1 mid-travel
```

3 Hz is `buildEasyWand`'s default `PulseHz`. RV1 exists so the rate can be traded
against the camera frame rate — pulse grouping needs frame period < pulse period
with margin.

The 555 is a free-running RC oscillator, so expect several percent of frequency
error and drift. That is fine: alignment fits an offset and rate per camera.
Do not treat the strobe period as a known constant.

## Current budget

| Load | Average |
|---|---|
| TLC555 | ~1 mA |
| Timing network | ~12 µA |
| D2 power indicator | ~1.2 mA |
| LEDs, 4 × 96 mA × 0.09 % | 0.35 mA |
| **Total** | **~2.6 mA** |

The timer, not the LEDs, dominates. 8×AA gives roughly 900 hours.

## Headroom — this is what forces 12 V

At 96 mA per marker:

| Term | V |
|---|---|
| 4 × Vf (850 nm at 96 mA) | ~7.4 |
| R2 sense | 0.65 |
| Q1 Vce, minimum to regulate | 0.50 |
| D1 Schottky on the driver | 0.45 |
| Q1 Vds + wiring | 0.06 |
| C1 droop during the pulse | 0.31 |
| **Minimum pack voltage** | **~9.3 V** |

8×AA alkaline or a 3S LiPo. **Not a 9 V PP3** — it has neither the headroom nor
the internal resistance for this.

`C1 = 1000 µF` covers the worst case: 4 × 96 mA × 804 µs = 309 µC → 0.31 V droop.

## Setting the LED current

`R2` on each marker board. Same value on all four.

| R2 | I_LED |
|---|---|
| 4R3 | 151 mA |
| **6R8** | **96 mA** (fitted) |
| 8R2 | 79 mA |
| 13R | 50 mA |

`R3` is a DNP footprint in parallel with `R2` for trimming upward. Measure across
`TP1`–`TP2` with a scope during a pulse: `I = V / R2`.

96 mA is 2.5× the 38.2 mA DC point in `LED_characterisation.xlsx`, at 0.09 % duty
— far inside any datasheet pulse rating. Q1 dissipates ~0.33 W *during* the pulse
and 0.3 mW average.

## Two parts you must not substitute casually

**Q1 (driver).** Must be rated **Vgs ±20 V**, because the gate swings the full
12 V rail. `DMP3098L-7` is ±20 V. Common SOT-23 P-FETs — AO3401A, Si2301 — are
only ±8…12 V and will fail.

**D1–D4 (marker).** The footprint is still **blank**. `LED_characterisation.xlsx`
ranks Kingbright 25 mm best (σ 0.015 px at 1186 µs, against XL's 6057 µs) but
records no MPN — fill in the `MPN` field and set the footprint before layout.
Its only failing was fill 0.82 against the 0.85 gate, which Test G (thicker wall)
is meant to fix.

D1–D4 are marked **DNP**, and that does *not* mean the board works without them.
It means a turnkey assembler skips them so the emitters can be hand-soldered
afterwards, with the standoff to the diffuser set by hand. DNP excludes a part
from the BOM and the position files, **not from the PCB** — the pads still have
to be there, so the package is needed before layout regardless.

`R3` on the same sheet is DNP in the ordinary sense: bare pads, genuinely not
fitted. Do not conflate the two.

## Still open

- **LED part number.** See above. Everything else on the marker board is fixed.
- **PCB layout.** Neither board is laid out yet. On the marker board, space
  D1–D4 as widely as the 20/25 mm shell allows and aim them **tangentially**,
  not radially — `NOTES.md` attributes the ring artefact to beam angle, not
  diffuser diameter.
- **Ambient IR.** Regime B needs a ~28 ms exposure, and Test F (`F_Ambient`) has
  not been run. If the background floods it, the fallback is Regime A — short
  exposure, long pulse — which this board can also produce by winding RV2 to its
  maximum, though duty then rises and the overdrive headroom goes away.
- **Which side D goes on.** Settled: the frame defines the *calibration* XY
  plane, not an electrical ground, so every marker keeps its own ground wire.
  But easyWand takes origin, +X and +Y and lets **Z follow the right-hand rule**,
  so `A→C × A→D` must come out pointing **up**. Lay the rig flat on the floor,
  stand at A looking down the arm toward C, and D must be on your **left**. Put
  D on the right and Z points into the floor: every camera then solves below
  ground level, mirrored, and it will not look like a sign error — it will look
  like a bad calibration.

## Verification

Both schematics pass with zero ERC violations, and the netlists have been checked
by hand against the intended circuit:

```bash
kicad-cli sch erc --severity-all -o /tmp/erc.rpt WandStrobeDriver/WandStrobeDriver.kicad_sch
kicad-cli sch export netlist --format kicadsexpr -o /tmp/n.net WandMarker/WandMarker.kicad_sch
```
