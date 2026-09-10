# Base station: time sync, drop and latency accounting

Four files, two on the base and one on each camera.

| file | runs on | job |
|---|---|---|
| `../timeSync.py` | camera (OpenMV) | monotonic clock + non-blocking resync client |
| `syncServer.py` | base | sync responder + telemetry sink + live stats |
| `fitClocks.py` | base | offline drift fit, drop and latency report |
| `simCameras.py` | base | fake N cameras, for shaking the base down |
| `testAccounting.py` | base | invariants a short live run cannot reach |

## What this is for

A **common time origin** for labelling, file naming and post-hoc alignment,
good to roughly 1 ms over WiFi after the drift fit.

It is **not** the alignment mechanism. Strobe-edge row reading is ~21 us
(NOTES.md, "The rolling shutter is also the solution") against ~1 ms here.
Three orders of magnitude. Use this to label frames and to prove the link is
healthy; use the strobe to align them.

## Running a session

```bash
python3 Base/syncServer.py --cams 1-6 --out runs/2026-09-10 --status 5
```

`--cams` takes `1-6`, `1,2,5`, or any mix. Cameras not in the list still get
served and logged, with a warning; expected cameras that never appear are
reported as `NO TRAFFIC`, and a camera that goes quiet mid-run is flagged
`SILENT Ns` in the live table.

Then, after the run:

```bash
python3 Base/fitClocks.py runs/2026-09-10 --fps 200
```

## On the camera

Four lines into the tracker:

```python
import timeSync
ts = timeSync.TimeSync(CAM_ID, HOST_IP)
ts.startup_sync(set_rtc=True)      # blocking, ONCE, before the loop

while True:
    t_us = ts.poll()               # replaces time.ticks_us()
    ...
    send(seq, t_us, tracks)
```

`ts.poll()` must replace `time.ticks_us()` in the timestamp path, not sit
alongside it. `ticks_us()` wraps at 2^30 us = **17.9 minutes**, so absolute
stamps taken from it fold back on themselves partway through a long trial.
`poll()` returns an unbounded monotonic count; masked to 32 bits on the wire it
wraps every 71.6 minutes, and `syncServer.py` unwraps it.

The other side of that: `poll()` only sees the wrap if it is called more often
than every ~8 minutes. At tracking rates that is every frame. Do not leave the
tracker paused for nine minutes and resume.

Call `ts.close()` on shutdown to flush the SD log.

## The clock is never stepped during a trial

Stepping puts a discontinuity in the middle of the data that no filter can
undo. Samples are logged raw; the fit happens on the base where the whole
series is visible at once. The only deliberate step is `startup_sync(set_rtc=True)`,
before the first frame.

With a coin cell on the VBAT pads the RTC survives a power cycle, so a camera
that reboots mid-session comes back with usable wall time and can name files
before its first resync lands. Its **monotonic** counter does restart at zero,
which `fitClocks.py` detects as a backward step and fits as a separate segment.

## Reading the output

- **`drift_ppm`** — crystal error against the base. Cheap parts are +/-30 ppm.
- **`start_only_error`** — what a single sync at t=0 would have accumulated by
  the end of the run. This is the column that justifies resyncing at all.
- **`worst_run`** — longest consecutive-loss run. Ten scattered singles are
  survivable; one run of ten is a hole the length of a gesture. Judge the link
  on this, not on mean loss.
- **`spread_ms` (p95-p50) and `ia_p99`** are exact. The absolute latency
  columns are differences between two clocks this fit aligned, so they carry
  the `rms_us` uncertainty. Read the spread, not the absolute.
- **Pairwise skew** is what matters for multi-view. A drift common to all
  cameras largely cancels in triangulation; a relative one does not.

## Testing without cameras

```bash
python3 Base/syncServer.py --cams 1-6 --out /tmp/simrun --status 0 --duration 215 &
python3 Base/simCameras.py --cams 6 --fps 200 --loss 0.004 --seconds 205
python3 Base/fitClocks.py /tmp/simrun --fps 200
```

`simCameras.py` imports the real `timeSync.py` under a MicroPython shim, so it
exercises the firmware module that will ship rather than a copy of it. Each
fake camera gets its own crystal error, printed at startup, so the fit can be
checked against known truth. `--tick-mod` shrinks the 2^30 wrap so wrap
handling can be tested in a 60 s run.

Drift is a slope, and its uncertainty goes as
`sigma_offset * sqrt(12) / (span * sqrt(n))`. At WiFi's ~0.5 ms asymmetry that
means a 60 s run with 4 samples resolves drift to only about +/-50 ppm - worse
than the crystal error being measured. Hence `WARMUP_PERIOD_MS`: 12 samples in
the first 25 s, then the 20 s cadence. **Short trials need the warmup; do not
raise `WARMUP_SAMPLES` without checking `rms_us` afterwards.**

## Ports

| port | traffic |
|---|---|
| 7007/udp | telemetry, cameras -> base |
| 7008/udp | sync request/response/report, both ways |
| 7100+cam_id/udp | camera's bound source port |

Deliberately not UDP/123. Binding 123 needs sudo, macOS's `timed` may hold it,
and NTP's protocol baggage buys nothing when both ends are ours. Same maths
(Cristian's algorithm), no root, and the round-trip time comes back so bad
samples can be discarded.

One port per camera on the base would not help: a port is a demultiplexing
integer, not a queue, and every packet crosses the same radio and the same
socket buffer regardless. Six cameras at 200 fps is 1200 packets/s and about
0.6 Mbit/s - the AP's collision domain is the ceiling, not the port count.

## Validation

`simCameras.py` gives each fake camera a known crystal error, so the fit can be
checked against truth. Six cameras, 200 fps, 0.4% burst loss, 180 s, 5 s resync:

| cam | true ppm | fitted ppm | error |
|---|---|---|---|
| 1 | +37.0 | +34.75 | 2.25 |
| 2 | -12.5 | -12.29 | 0.21 |
| 3 | +4.0 | +3.80 | 0.20 |
| 4 | -28.0 | -27.90 | 0.10 |
| 5 | +19.5 | +19.98 | 0.48 |
| 6 | -6.0 | -4.22 | 1.78 |

Run with `--tick-mod 67108864`, forcing a `ticks_us` wrap every 67 s, the same
run produced zero false restarts.

**Accuracy depends on span, hard.** The same setup over 78 s instead of 180 s
gave 4-7 ppm errors rather than 0.1-2.3, because slope uncertainty goes as
`sigma_offset * sqrt(12) / (span * sqrt(n))`. Do not read a drift figure from a
one-minute run and believe it. `rms_us` is the column that tells you whether to.

`testAccounting.py` covers what a live run cannot reach in reasonable time: the
16-bit seq wrap (5.5 min at 200 fps), the 32-bit `t_us` wrap (71.6 min), a
reordered packet, and a genuine reboot. Case 3 caught a real bug - a reordered
packet's timestamp goes backwards, the reboot detector read that as a restart,
and a restart clears the gap bookkeeping, so the losses being measured
disappeared. Run it after touching anything in `CamStats`.
