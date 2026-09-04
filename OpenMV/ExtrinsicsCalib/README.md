# ExtrinsicsCalib

Extrinsic calibration capture for 7 OpenMV RT1062 cameras, producing easyWand6
input files. Standalone — shares no imports with `kalmanFilter.py`, though the
geometry and centroid maths are lifted from it unchanged so the two agree on
what a sensor pixel is.

## Files

| File | Runs on | Does |
|---|---|---|
| `extrinsicsCalib.py` | each camera | full-field scan, orders markers, streams UDP |
| `collectCalib.m` | PC | receives and logs packets |
| `buildEasyWand.m` | PC | groups packets into pulses, writes easyWand CSVs |
| `writeCameraProfile.m` | PC | writes the 7-row camera profile |

## The rig

```
D
|
|                 D arm perpendicular at A, length >= |AC|/3
A-------------B---------C
<-----2/3-----><--1/3--->
```

D hangs off **A**, because A is the origin and the axis file needs +X and +Y as
directions measured from it.

Four active IR markers, all strobed **simultaneously** by the 555 driver. B sits
off-centre so `|AB|/|BC| ≈ 2`. That asymmetry is what lets every camera label
A/B/C/D identically without talking to each other.

`order_markers()` works as follows. The point furthest from the line through the
other three is **D**. Of the remaining collinear triple, the middle one is **B**
and the ends are A and C. **A is the end farther from B.**

Collinearity and betweenness are preserved exactly under perspective projection,
so identifying D and B is rigorous. The A/C call uses `|AB|/|BC|`, which is only
affine-invariant, so three gates reject frames where it can't be trusted:

| Gate | Rejects |
|---|---|
| `COLLINEAR_MAX_RESID = 0.15` | B not on the line — not the rig |
| `D_OFFLINE_MIN = 0.10` | D projected onto the line, so D/B aren't separable |
| `AC_RATIO_MIN = 1.2` | `\|AB\|/\|BC\|` too close to 1 to call |

Monte-Carlo over 100,000 random poses with perspective projection and centroid
noise: **0.000% mislabelled, ~15% of frames rejected.** A rejected frame sends
`n_valid = 0` and becomes NaN in the CSV, which is the honest outcome — a wrong
label would silently corrupt the calibration, a NaN costs you one observation.

**Design constraint: keep the camera at least 2× the wand length away.** Below
that the wand's depth extent approaches its distance, the affine assumption
collapses and mislabels start getting through the gates:

| distance / wand length | mislabelled | rejected |
|---|---|---|
| 1.0 | 3.9% | 26% |
| 1.5 | 4.8% | 30% |
| **2.0** | **0.00%** | 26% |
| 3.0 | 0.00% | 15% |
| 10.0 | 0.00% | 15% |

This sits comfortably inside the usual advice to make the wand a quarter to a
third of the shortest working dimension.

## Order of operations

### 1. Measure `t_readout`

`MODE = "readout"` on one camera. Set a short exposure, strobe the markers, read
`band_frac`. Then `t_readout = EXPOSURE_US / band_frac`.

This is the number everything else depends on. A rolling shutter only has an
instant when *all* rows are integrating if `t_exp > t_readout + t_pulse`. Below
that, the strobe lights a band and markers outside it vanish — and markers
straddling the band edge get a **biased centroid**, which is worse because it
doesn't look like a failure.

Set `EXPOSURE_US` above the measured `t_readout`, then pick a frame rate giving
an acceptable hit rate:

```
hit rate  ≈  1 − (t_readout + t_pulse) / t_frame
```

With 7 cameras all needing to catch the same pulse, that's raised to the 7th
power. At `t_readout/t_frame = 0.25` you get 0.75⁷ = 13% of pulses usable — so
run **well** below the sensor's maximum frame rate. Dropping to `csi.QVGA`
halves `t_readout` and helps directly.

### 2. Set the exposure, then tune the LEDs

`MODE = "expose"`. The exposure is now fixed long, so **adjust the LED trimpot,
not the camera**. Target peak 200–230 with `sat = 0` and `blobs = 4`.

Long exposure raises the ambient floor by roughly the exposure ratio (700 µs →
30 ms is ~40×). You will need IR bandpass filters or a dark room. Then re-run
`kalmanFilter.py MODE="floor"` and copy `REFINE_FLOOR`, `CENTROID_SIGMA` and
`AREA_NOMINAL_SENSOR` across — **all three from the same row**.

### 3. Set `CAM_ID` 1–7 and flash all seven

Nothing else differs between cameras.

### 4. Record

```matlab
collectCalib('wandRun.mat', 120);     % sweep the wand through the whole volume
collectCalib('axisRun.mat',  10);     % rig held STILL in its reference pose
```

For the axis run, place the rig deliberately — flat on the floor if you want the
floor to be your XY plane.

### 5. Build the easyWand files

```matlab
buildEasyWand('wandRun.mat', 'wand', 'PulseHz', 3);
buildEasyWand('axisRun.mat', 'axis', 'PulseHz', 3);
writeCameraProfile('NumCams', 7, 'FocalLengthMm', 2.8);
```

Everything lands in `easyWandIn/`.

### 6. Load into easyWand6

| easyWand6 field | File |
|---|---|
| Camera profile | `cameraProfile.txt` |
| Paired points | `wandPoints.csv` |
| Inter-point distance | `\|A−C\|` from OptiTrack |
| Unpaired points | `unpairedPoints.csv` |
| Alignment (axis points) | `axisPoints.csv` |

## File formats produced

For N = 7 cameras:

- `wandPoints.csv` — 28 columns, one row per pulse. **Point-major**:
  all cameras' pt1 (marker A), then all cameras' pt2 (marker C).
- `unpairedPoints.csv` — 14 columns, one row per marker observation, 4 per pulse.
- `axisPoints.csv` — 14 columns, 3 rows: A = origin, C = +X, D = +Y.
- `cameraProfile.txt` — `id f_px W H cx cy 1 0 0 0 0 0`, one row per camera.

`NaN` marks a camera that didn't see the point. Dump everything — easyWand
selects usable rows itself and reports which it kept.

## Two things that will bite you

**Coordinates are SENSOR pixels.** `image_to_sensor()` folds the readout-window
offset and framesize ratio back into the full sensor frame, exactly as
`kalmanFilter.py` does. So the camera profile must describe the **full sensor
array** (2592×1944), not the VGA framesize you capture at, and `f_px` must be in
sensor pixels. Origin is upper-left, matching easyWand6.

**Pulse grouping uses host arrival time.** The camera clocks are free-running and
unsynchronised, so `t_us` is carried for diagnostics only. Grouping works because
every camera that catches a pulse reports it within one frame period, while
consecutive pulses are a whole pulse period apart. This **requires frame period <
pulse period with margin** — check the diagnostics `buildEasyWand` prints, and
watch the cameras-per-pulse histogram.

## Why no windowing or Kalman filter here

`kalmanFilter.py` windows to hold ratio at 1.0 for precision. This does the
opposite deliberately:

- Full-field scanning keeps `t_readout` **constant**, so the strobe hit rate is
  predictable. Adaptive windowing makes both vary frame to frame.
- Calibration needs markers in the image corners more than it needs precision.
  At VGA over the full sensor, ratio ≈ 4 and centroids land around 0.2–0.6 sensor
  px — in line with the easyWand test cases (0.15–1.8 px).
- At a 3 Hz strobe the marker moves too far between observations for
  frame-to-frame association to mean anything. `order_markers()` labels each
  frame independently from rig geometry, so no tracking is needed.
