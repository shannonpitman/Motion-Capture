# Marker characterisation protocol

How to run a valid measurement session, and every systematic we have found the
hard way. Written against the OpenMV RT1062 / OV5640 (2592x1944) rig.

`NOTES.md` holds the running commentary and results; this file holds the method.
If the two disagree, this file is wrong and should be fixed.

---

## 0. Order of operations

Do these in order. Later steps are invalid if an earlier one is skipped, and in
most cases **nothing downstream can detect the omission from the data alone**.

1. **Lens focus** - set, verify, lock. (Section 1)
2. **Supply** - known, measured, logged. (Section 2)
3. **Ambient** - characterise the room at the time of measurement. (Section 3)
4. **Instrument configuration** - fix and hold. (Section 4)
5. **Controls** - the per-session checks. (Section 5)
6. **Measurement**. (Section 6)

---

## 1. Lens focus - PREREQUISITE, and the one most easily missed

Focus must be **set, verified and locked before intrinsics are captured**, and
re-verified before extrinsics. This belongs in the GUI as a formal step.

### Why it is a prerequisite

- **It silently corrupts range characterisation.** A near-focused lens makes the
  blob hollow out and grow with distance. That mimics, and partly cancels, the
  1/d^2 falloff: the exposure-vs-distance curve flattens and the apparent
  detection range is wrong. Not recoverable from the data afterwards.
- **It corrupts intrinsics.** Defocus degrades corner/blob localisation, so the
  distortion coefficients absorb what is really a focus error.
- **Focus must not move between intrinsics and extrinsics.** Changing it shifts
  the effective focal length and silently invalidates the intrinsics. If the
  ring is not mechanically locked it *will* drift when the camera is handled.
- **All cameras must be focused at the same working distance**, or per-camera
  centroid precision differs and triangulation is weighted wrongly.

### Procedure

1. Place a marker at mid-flight-space distance (~4-5 m for a 9x7x4 m volume).
2. Run the focus readout at fixed exposure (`focus.py`).
3. Adjust for **minimum spot size**. Then lock the ring mechanically (grub
   screw / paint / tape) and **record the locked position**.
4. Capture intrinsics. Do not touch the ring again.
5. Before extrinsics, re-run step 2 as a VERIFY-ONLY check against the recorded
   values. If they have moved, the ring moved: recapture intrinsics.

### Which metric to focus on

Use **brightness-independent** spot-size measures. Both are ratios of the
profile to itself, so they hold still when the source dims:

| metric | definition | target |
|---|---|---|
| `ew` (equivalent width) | flux / peak, px^2 | minimise |
| `rms_r` (RMS radius) | sqrt(sum w*r^2 / sum w), px | minimise |

**Do NOT focus on `area`.** It counts pixels over a fixed threshold (80), so it
shrinks whenever the source dims - it cannot tell defocus from a flat battery.
We were caught by exactly this: peak and area fell *together*, which no focus
change can produce, and it turned out the camera had been nudged.

`peak` is a valid focus metric only while it is unsaturated and the source
brightness is constant. It pins at 255 quickly once focus improves.

### The defocus signature, for diagnosis after the fact

`fill` = centre brightness / rim brightness. If `fill` **falls with distance and
crosses below 1.0**, the blob is an annulus - centre dimmer than rim - and the
lens is focused near. A source going point-like does the opposite: it converges
on the PSF and gets *more* peaked, `fill` well above 1.

Corroborating signs: the exposure-vs-distance exponent collapses (we saw ~1.53
fall to 1.05), and `area x d^2` rises because defocus adds apparent size that
partly cancels 1/d^2.

Observed at 5 m when the fix was applied, same exposure:
`fill` 0.73 -> 1.46, `area` 571 -> 316, diameter 27.0 -> 20.0 px.

---

## 2. Supply

- **Log the pack voltage, measured under load, for every session.** Not "a 9 V
  battery" - the number.
- With only ~2.6 V of headroom above 4x Vf, current - and therefore brightness -
  is hypersensitive to supply sag. A marker measured **2.4x dimmer** on a lab
  cell of unknown charge than on a fresh one at matched geometry.
- Consequence for the driver design: this is the argument for a **constant-
  current sink rather than a series resistor**.
- **Battery state is an uncontrolled variable until it is measured.** A dying
  cell has already voided one row by inflating sigma 3x and splitting the blob,
  and it was initially misdiagnosed as marker structure.

### Battery control (run once per session, ~90 s)

Fix the exposure, hold, and log peak and area. Expected on a good cell:

- `peak` flat to within ~1% - scatter should be random, not monotonic.
- `area` declining monotonically ~0.6%/min. This is **LED junction heating**,
  not discharge - peak does not sag with it. Small enough to ignore for sigma;
  do not mistake it for a flat battery.

### If the supply is of unknown charge

The session is still worth running, because most outputs are measured at matched
peak and are therefore supply-independent (sigma, fill, lobe, w/h, area x d^2).
Only the *absolute exposure* carries the unknown scale factor. So:

- Record each row's exposure **normalised to that block's reference row**
  (`C_Range` column S). The ratio is supply-independent.
- Bracket the session: measure the reference row first and last. Stable means
  the factor held; a sudden step is visible as a kink in an otherwise smooth
  sweep.
- **A detection is a valid lower bound. A non-detection is provisional** and
  must be flagged supply-limited until re-checked on a known-good pack.

---

## 3. Ambient

Characterise the room *at the time of measurement*, with the LEDs off, over an
exposure ladder. The question is not "how bright is it" but **"does the
background scale with exposure?"**

- Flat with exposure = sensor pedestal, not light. Harmless.
- Rising with exposure = real IR. This is what sets the ceiling.

Measured in the arena at night: **flat at 3.3-3.5 counts from 85 us to 100 ms**,
first real signal above 100 ms, still only 10 counts at 200 ms against
`THRESH_LO` 80. No practical ceiling below ~200 ms.

Daylight, same room: ceiling ~14.5 ms in frame, ~40 ms away from the windows.
A window light that ruined a 3.5 m row in daylight - the targeting ran to
101 702 us and locked onto it, reporting blobs=3, w/h=1.59, lobe=0.787 - reads
5 counts at the same exposure at night.

So: **the night arena needs no blackout**, and the usable integration window is
more than 13x larger than in daylight. Re-run the ladder each session; do not
assume yesterday's ceiling.

---

## 4. Instrument configuration - fix these and hold them

| setting | value | why |
|---|---|---|
| readout window | **forced VGA 640x480, ratio 1** | see below |
| `auto_gain` | off, `gain_db=0` | gain changes are a hidden covariate |
| `auto_exposure` | off, explicit `exposure_us` | |
| `THRESH_LO` | 80 | blob detection floor |
| `REFINE_FLOOR` | 50 | background subtracted before weighting |
| `SAT_LEVEL` | 250 | |
| target peak | **205-215**, unsaturated | matched-peak is what makes rows comparable |

### The readout window changes sigma by 35%

Same marker, same placement, back to back at 3 m:

| | QVGA 320x240 | VGA 640x480 |
|---|---|---|
| exposure | 30 988 us | 32 215 us (+4%) |
| area | 1093 | 1186 (+8.5%) |
| **sigma_RMS** | **0.0233** | **0.0152 (-35%)** |

So the **signal is window-independent** - there is no exposure scaling to apply
between modes, the 4% is inside placement noise - but the **noise is not**.
Never let the window size change within a study.

The old auto-size rule (drop to QVGA below a 60 px blob) existed to avoid
black-level compression in large blobs. That does not bite in VGA: a 103 px blob
is 2.7% of a VGA frame, well under the ~10% where compression starts. Force VGA.

### This is live in the tracker

`kalmanFilter.py` runs an adaptive ladder (QQVGA / QVGA / VGA) and switches
rungs as the blob grows and shrinks - exactly what a drone moving in depth does.
`detection_R` models `R = CENTROID_SIGMA^2 * (AREA_NOMINAL_SENSOR / npix)`,
i.e. sigma^2 proportional to 1/N.

Between the two runs above, `npix` went 1256 -> 1350, just 7.5%, so that model
predicts a 3.6% change in sigma. Measured: 35%. **There is a framesize term
missing from R**, worth ~2.3x in variance at a rung boundary.

Direction of the resulting error depends on where `CENTROID_SIGMA = 0.0544` was
tuned. `ledCharacterise.py` sets `MEASURE_RES = csi.QVGA`, so the calibration
tooling measures in QVGA, making 0.0544 a QVGA-flavoured - conservative - number
relative to VGA. That is conservative **by accident**: retune it in VGA and the
filter becomes overconfident by that 2.3x whenever the tracker drops a rung.

Fix: either pin `PIN_TRACKING_RES`, or calibrate sigma per rung. **QQVGA is on
the ladder and has never been characterised at all.**

### Exposure gotchas

- **Re-apply the exposure AFTER `IOCTL_SET_READOUT_WINDOW`.** Programming the
  readout window resets it - 25 000 us requested came back as 10 880 us - and at
  range that silently drops the marker below `THRESH_LO`, so no blob is found
  and the run looks like a dead marker.
- Exposure is **quantised to the row time (~21.3 us)** and the achievable grid
  depends on the readout window. Targeting must detect when the grid straddles
  the target band and stop, rather than oscillating.
- At full-sensor search resolution the ratio is ~4, so a well-focused distant
  blob can be only 2-3 search pixels across and brushes
  `pixels_threshold=6`. Use `area_threshold=2, pixels_threshold=2` **for the
  search step only**, or the window lands in the wrong place.

### Targeting must use the mean, not the max

The convergence sampler must return the **mean of per-frame peaks over ~10
frames**, matching what the 200-frame measurement reports. Taking a max over
frames biases high and lands the measured mean below target. This was a real
bug; every row taken before the fix sat low.

---

## 5. Per-session controls

Run all of these. They are cheap and each has caught a real error.

- [ ] **Ambient ladder**, LEDs off, exposure ladder to at least 200 ms. (Sec. 3)
- [ ] **Supply voltage** under load, written down.
- [ ] **Battery hold**, 90 s at fixed exposure. (Sec. 2)
- [ ] **Focus verify** against the recorded locked values. (Sec. 1)
- [ ] **Reference row** measured first and last, to bracket drift.
- [ ] **Replicate** at least one row - re-seat the marker and re-measure.

### Placement repeatability

Measured at 90 deg, the worst case, n=3 re-seats at 3 m: exposures 30 090 /
32 067 / 30 988 us, **1 sigma = 3.2%**; area 1105 / 1108 / 1093, **0.7%**.

Orientation is *not* a dominant error source, even at 90 deg where dI/dtheta is
steepest. If a discrepancy is much larger than ~3%, look elsewhere - it is not
the placement.

### Reading anomalies

- **Geometry steady + sigma inflated = rig disturbance, not marker.** If `area`,
  `w/h`, `fill` and `lobe` are unchanged but sigma jumps, something shook. Re-run
  rather than theorising. This rule has been right twice.
- **Peak and area falling together** cannot be a focus change - focus conserves
  total flux and trades peak against area. Both falling means the source dimmed
  or the camera moved.
- A saturated blob reports nonsense shape metrics (33 blobs, w/h 23.2,
  sigma 1.22 in one case). That is **saturation row-banding**, not structure -
  distinct from the Bayer CFA pattern and from PWM banding. Withhold shape
  metrics from any saturated attempt; `stripeCheck.py` separates the causes.

---

## 6. Measurement

1. Locate the marker at full-sensor search resolution (loosened thresholds).
2. Programme a **VGA 640x480 ratio-1** readout window centred on it.
3. Re-apply the exposure.
4. Close the loop to **peak 205-215**, unsaturated, using mean-of-per-frame-peaks.
5. Measure **200 frames**; require at least 20 valid.
6. Emit one `RESULT` line: exposure, peak, peakmax, satpx, satmax, framessat,
   area, npix, blobs, w/h, fill, lobe, sigma_u, sigma_v, sigma_RMS.

### Metric definitions

| metric | meaning |
|---|---|
| `area` | blob pixels above `THRESH_LO`, in **sensor** px (ratio 1) |
| `npix` | pixels above `REFINE_FLOOR` contributing to the centroid |
| `blobs` | count at `merge=False` - >1 means the marker has fragmented |
| `w/h` | bounding-box aspect; 0.9-1.1 is round |
| `fill` | centre (r<0.30R) / rim (0.60R-0.90R) mean brightness |
| `lobe` | 8-sector angular modulation, (hi-lo)/(hi+lo) |
| `sigma_*` | centroid scatter over the run, sensor px |

`fill` and `lobe` are **exposure-dependent** - compare them only at matched
peak. `fill` also goes noisy once the blob is small (below ~12 px the centre
sample is a pixel or two wide); prefer `ew` / `rms_r` there.

### Blend gate

`blobs == 1` AND `0.9 <= w/h <= 1.1` AND `fill >= 0.70`.

The 0.70 threshold is measured, not chosen by eye: across 16 builds, sigma
saturates above `fill` ~0.70. An earlier 0.85 gate was wrong and rejected good
caps.

---

## 7. Recording conventions

- **Freeze superseded runs** in their own sheet (`A_Bench_run1`, `C_Range_run1`)
  with a header saying what changed and which columns remain comparable.
- **Normalise exposures** to a per-block reference row so the supply factor
  divides out. Absolute exposures are only meaningful with a logged voltage.
- **VOID a row rather than deleting it**, with the reason. Voided rows are
  evidence about the method.
- **Non-detections are provisional** unless every ceiling - ambient, supply,
  focus - was verified at the time.
- Never insert a column without re-checking the formulas. A previous insert
  shifted only the first half of every range (`AVERAGE(A_Bench!G5:F5)`) and the
  damage was silent.

---

## 7a. Intrinsic calibration

Full detail in `intrinsics/README.md`. The points that are easy to get wrong:

### Capture in the tracker's pixel frame

Intrinsics are in pixels and only valid in the frame they were measured in.
`framesize(csi.VGA)` alone is **not** a 1:1 sensor crop - it is the full sensor
FOV downscaled by **4.050**. The tracker reports centroids in **sensor** pixels.

So capture at `FULL_RES = True` (`WQXGA2`, 2592x1944, ratio 1.000) and the
intrinsics land in the tracker's own frame with nothing to rescale. If VGA is
used instead, fx, fy, cx, cy must be multiplied by 4.050; k and p are
normalised and do not scale. `calibrateIntrinsics.m` warns if the frames are
not sensor-sized.

### Coverage beats fill

**k1, k2, k3 are constrained only by corners far from the principal point.** A
board that stays near frame centre yields three radial coefficients fitted to
nothing - with plausible-looking standard errors, and no warning from MATLAB.

Measured on cam2: an A3 / 40 mm board at 1 m occupied 2.2% of the frame, with
corners spanning radius 47-157 px against a frame corner at 400 px - the inner
39% of the radial field. Unusable for distortion.

What matters is **where the corners land**, not how much of the frame is
filled. Photograph the board into all nine cells of a 3x3 grid and push it hard
into the four corner cells. Full-resolution capture does NOT help coverage
(fill fraction is scale-invariant); it improves corner precision only.

### Lens and depth of field

Measured on cam2: f ~= 300 px in 640x480 space (~1215 sensor px, ~1.7 mm),
**HFOV ~94 deg**. Hyperfocal is ~0.5 m loose, ~1.0 m strict, so **focused at
~1 m the lens is sharp from about 0.5 m to infinity**.

That is the important result: **one focus setting serves both the checkerboard
and the arena.** There is no need to refocus between calibration and the range
test, which is what would otherwise invalidate the intrinsics (Section 1).
Verify it rather than assuming - run `focusCheck.py` at ~1.5 m and at 5 m and
confirm `ew` / `rms_r` are acceptable at both.

### Exposure and gain

**Auto exposure is unusable** - it pins at the frame-rate ceiling (21 450 us at
VGA) and still underexposes (mean 52.8, max 161 against a 200-240 target).
Set exposure explicitly.

Long exposure blurs a handheld board and **blur biases corners inward** - a
systematic error, not noise. Trade exposure for gain: measured at a fixed
8000 us, gain 0/6/12/18 dB gave max 77/131/198/255, so **12 dB buys a 4.2x
shorter exposure** (8000 us at 12 dB == 33 800 us at 0 dB). Costs ~2x in
shot-noise SNR, which corner fitting tolerates far better than blur.

On a 94 deg lens the frame corners vignette noticeably, and those are exactly
the placements that matter most. Check boards near the frame edge still reach
~150 max.

### ROLLING SHUTTER IS THE DOMINANT ERROR SOURCE - BRACE THE BOARD

The OV5640 is a rolling-shutter sensor and a **full-res readout takes 74 ms**.
A board spanning 1000 of the 1944 rows is therefore scanned over ~38 ms, so any
motion during readout SHEARS the board. This is not blur - it is a geometric
distortion of the target itself.

Measured on the first cam2 run (40 frames, hand-held, A3 board / 40 mm squares):

  linear shear term vs per-image reprojection error   r = +0.84

  best 5 images : shear 0.03-0.19 px -> error 0.37-0.44 px
  worst 5 images: shear 0.29-3.68 px -> error 0.90-1.93 px
  board spanned 492-1278 rows = 18.7-48.7 ms of readout

Everything else was ruled out first, and none of it explains the error:

  board flatness  mean residual per board point equals its own s.e.m.
                  (ratio 1.0x) - no systematic view-independent pattern
  brightness      r = +0.19
  sharpness       r = -0.13
  reach           r = -0.07

**Budget.** To hold shear under 0.5 sensor px across a ~38 ms readout the board
must move slower than about 13 sensor px/s. That is roughly ten times steadier
than a hand can manage. **The board must be braced** - a stand, a clamp, or
leant against something - and moved between shots, not held.

Note the two budgets are different and the tighter one wins:
  BLUR  - smear within one exposure (8-40 ms), softens corners
  SHEAR - motion across the 74 ms readout, deforms the target (~10x tighter)

### Capture must be verified at full resolution

The preview is a 4x downscale, and downscaled detection is more forgiving than
the real thing - a frame can pass the live gate and still fail at full res.
Combined with ~0.3 s of trigger latency, 9 of 40 frames in run 1 were lost,
5 of them because the board drifted off the frame edge between the approved
preview and the actual capture.

So: (a) require a margin from the frame edge of at least 4% of the frame, and
at least twice the allowed drift; (b) after saving, re-run detection on the
saved full-res file and DELETE it if it fails, prompting a redo. The set is
then good by construction rather than by inspection.

### cam2, measured 2026-09-10 (run 1, hand-held - PROVISIONAL)

  31/40 detected, RMS 0.756 px (0.466 on the best 16). Target is 0.25.
  fx, fy   2323.3 +/- 1.2 , 2323.7 +/- 1.4   (ratio 0.9998 - square pixels)
  cx, cy   1162.8 +/- 2.1 , 875.2 +/- 2.0    (frame centre 1296, 972)
  k1 -0.464   k2 +0.272   k3 -0.100   p1 +0.0011   p2 +0.0016   (all significant)

Stable but not final: across 12 random half-splits fx varied +/-6 and cx +/-10,
and pruning 31 -> 16 images moved fx by 0.2% while nearly halving RMS. So the
parameters are well determined; the residual is inflated by sheared frames.

**Field of view: quote the distorted figure, not the pinhole one.**
  pinhole 2*atan(W/2fx)      H 58.3 deg, V 45.4 deg     <- MISLEADING
  real rays (undistorted)    H 67.3 deg, V 49.3 deg, diagonal 77.2 deg
  lens datasheet             H 70.8 deg, V 55.6 deg
The paraxial focal length (3.25 mm at 1.4 um pitch) is LARGER than the "2.8 mm"
on the label because the lens has <-24% TV distortion; the datasheet's own
figures are only self-consistent that way. We also see slightly less field than
the datasheet because the OV5640 is 1/4" (3.63 mm wide) and under-fills a lens
rated for 1/3".

**The principal point is genuinely off-centre by 133 x 97 px = 231 um.** That is
the scale of a re-seated sensor, and this camera had its sensor removed to fit
the visible-light filter. The small but significant tangential terms are the
matching signature of slight sensor tilt. Do not assume the principal point is
at the frame centre - it is 5% of the frame width away.

**Refocusing moves cx, cy more than it moves fx.** Screwing the lens from a 1 m
to a 1.9 m focus changes fx by only ~0.15% (~3.5 px) - small, but still 3x the
fit's standard error. The larger risk is that M12 threads have play, so turning
the lens decentres and tilts it, which shifts the principal point. This is the
mechanical reason for the lock-focus-before-intrinsics rule in Section 1.
Fitting a filter behind the lens also pushes focus back by about t/3 for
thickness t, so the lens must be re-set afterwards - expected, not a fault.

**Depth of field, corrected.** With f = 3.25 mm at f/2.0, hyperfocal is ~1.9 m.
Focused at 1 m the lens is sharp from roughly 0.65 to 2.1 m - which puts arena
markers at 5-8 m OUTSIDE it. Set focus to ~1.9 m and shoot the board at
1.2-2.0 m; one setting then serves both calibration and the range test.

**Coverage was the weakest part of run 1**: only 0.6% of corners reached beyond
80% of the radial field, max 89%. Tilt was good - median 40 deg, only 4 of 31
frames under 15 deg.

**Vignetting is still unmeasured.** Stacking the 40 frames as a pseudo
flat-field gave a 76% -> 11% falloff, but that cannot separate lens vignetting
from the fact that one torch lit only part of the room - the apparent
illumination centre landed at (1118, 589), nowhere near the principal point,
which shows the stack is dominated by scene lighting. **Test it properly:** fill
the frame with a blank, evenly lit wall and measure the radial profile. The
datasheet claims >70% relative illumination.

### Sequencing

Intrinsics depend on **focus**; extrinsics depend on **camera pose**. Intrinsics
survive the camera being moved, extrinsics do not. So:

    focus -> lock -> intrinsics -> range test -> extrinsics

Capture extrinsics **last**, after the physically disruptive work in the arena
is finished. Doing them first and then working around the rig invalidates them
silently. Note that a single camera can only give intrinsics - extrinsics needs
the second camera in place, or a known-geometry world target.

## 8. Known open items

- `QQVGA` rung never characterised; `R` has no framesize term.
- `REFINE_MAX_PX = 3000` silently disables subpixel refinement closer than
  ~3 m in `kalmanFilter.py`. Live bug, not a measurement artifact.
- `AREA_MIN_SENSOR = 120` still uncalibrated - needs the marker area at the
  furthest working distance.
- Motion smear ~1300 px/s per m/s of target speed at 4 m. **Provisional**; needs
  confirming against a known velocity, and it is the real ceiling on exposure
  at night now that ambient is not.
- Intrinsics not yet captured, so there is no px-per-mm scale and the
  camera-count decision is still open.
