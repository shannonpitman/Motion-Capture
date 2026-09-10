#!/usr/bin/env python3
"""fitClocks.py - Shannon Pitman

Post-processes a syncServer.py run directory. Three outputs:

  1. A per-camera clock model  server_ref_us = a + (1 + b) * cam_mono_us,
     so every camera's timestamps can be expressed on one common clock.

     SIGN CONVENTION. b is the mapping coefficient - the BASE's rate as seen
     from the camera. The camera's own crystal error is cam_ppm = -b * 1e6,
     because a camera clock running fast makes the base look slow. cam_ppm is
     what the reports print, since that is the number to compare against the
     +/-30 ppm a cheap crystal is specified for. b is what MATLAB should use
     to remap timestamps. Do not mix them up.
  2. An honest error budget for that model, and what a start-only sync would
     have cost instead.
  3. Drop and latency statistics per camera, and pairwise skew between them.

    python3 Base/fitClocks.py runs/2026-09-10

Stdlib only - no numpy - so it runs wherever the capture ran.

READ THE LATENCY NUMBERS CAREFULLY
Absolute one-way latency is known only to about the sync residual, because it
is computed by differencing two clocks that were themselves aligned by this
fit. The SPREAD is exact - jitter, p95-p50, and the shape of the tail owe
nothing to the clock model. So the p95-minus-median column is trustworthy in a
way the median column is not, and the minimum is the better estimate of the
true floor. Judge the link on spread, not on the absolute.
"""

import argparse
import math
import os
import re
import sys

FRAME_HINT_HZ = 200.0        # only used to phrase errors in frames


def read_csv(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            try:
                rows.append([int(x) if x else None for x in line.split(',')])
            except ValueError:
                continue
    return rows


def linfit(xs, ys):
    """Least squares y = a + b*x, centred first.

    Centring is not cosmetic. Raw x here is microseconds since camera boot,
    order 1e9, so sum(x^2) is order 1e18 while the variance being resolved is
    order 1e6 - the same catastrophic cancellation that made mode_measure print
    0.0000 in NOTES.md. Working in deviations keeps every term O(1)."""
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = 0.0
    sxy = 0.0
    for x, y in zip(xs, ys):
        dx = x - mx
        sxx += dx * dx
        sxy += dx * (y - my)
    if sxx == 0.0:
        return None
    b = sxy / sxx
    a = my - b * mx
    return a, b, mx, my


def robust_fit(samples, keep_frac=0.4):
    """samples: list of (t_cam, offset, delay).

    Two filters, in order. First keep only the lowest-delay fraction: the
    minimum-delay sample of a burst is the one whose path was least
    asymmetric, and asymmetry is the entire error term in Cristian's
    algorithm. Then reject residual outliers past 3 sigma and refit, which
    catches the occasional sample where both directions were slow together and
    the delay filter could not see it."""
    if len(samples) < 4:
        return None
    ordered = sorted(samples, key=lambda s: s[2])
    keep = ordered[:max(4, int(len(ordered) * keep_frac))]
    xs = [s[0] for s in keep]
    ys = [s[1] for s in keep]
    fit = linfit(xs, ys)
    if fit is None:
        return None
    a, b, _, _ = fit
    res = [y - (a + b * x) for x, y in zip(xs, ys)]
    sd = math.sqrt(sum(r * r for r in res) / len(res)) if res else 0.0
    if sd > 0:
        keep2 = [(x, y) for x, y, r in zip(xs, ys, res) if abs(r) <= 3 * sd]
        if len(keep2) >= 4:
            fit = linfit([k[0] for k in keep2], [k[1] for k in keep2])
            a, b = fit[0], fit[1]
            xs = [k[0] for k in keep2]
            ys = [k[1] for k in keep2]
    res = [y - (a + b * x) for x, y in zip(xs, ys)]
    rms = math.sqrt(sum(r * r for r in res) / len(res)) if res else 0.0
    return dict(a=a, b=b, n_used=len(xs), n_total=len(samples),
                rms=rms, worst=max(abs(r) for r in res) if res else 0.0,
                t_lo=min(xs), t_hi=max(xs))


def split_reboots(samples):
    """A camera that reboots restarts its monotonic clock at zero. Fitting
    across that discontinuity produces a nonsense drift, so split into
    segments wherever the camera clock steps backwards. With the coin cell on
    the VBAT pads the RTC survives, but the monotonic counter does not."""
    segs = []
    cur = []
    last = None
    for s in samples:
        if last is not None and s[0] < last:
            segs.append(cur)
            cur = []
        cur.append(s)
        last = s[0]
    if cur:
        segs.append(cur)
    return segs


def pct(v, p):
    if not v:
        return float('nan')
    k = (len(v) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def analyse_cam(run_dir, cam):
    out = {'cam': cam}

    raw = read_csv(os.path.join(run_dir, 'syncfit_cam%d.csv' % cam))
    samples = []
    for r in raw:
        if len(r) < 7 or any(x is None for x in r[:7]):
            continue
        _, t1, t4, t2, t3, delay, offset = r[:7]
        samples.append(((t1 + t4) // 2, offset, delay))
    out['n_sync'] = len(samples)
    out['segments'] = []
    for seg in split_reboots(samples):
        fit = robust_fit(seg)
        if fit:
            out['segments'].append(fit)
    if not out['segments']:
        return out

    # Telemetry: drops and latency, using the segment covering each packet.
    telem = read_csv(os.path.join(run_dir, 'telem_cam%d.csv' % cam))
    lat = []
    seqs = []
    ia = []
    prev_ref = None
    for r in telem:
        if len(r) < 7:
            continue
        recv_ref, recv_unix, seg, init_id, seq, t_mono, n_mark = r[:7]
        seqs.append((seg, seq))
        if prev_ref is not None:
            ia.append(recv_ref - prev_ref)
        prev_ref = recv_ref
        fit = pick_segment(out['segments'], t_mono)
        if fit:
            lat.append(recv_ref - (fit['a'] + (1.0 + fit['b']) * t_mono))
    out['rx'] = len(seqs)
    if seqs:
        # Sequence numbers either side of a camera restart are not comparable,
        # so loss is accounted within each segment and then summed.
        by_seg = {}
        for seg, sq in seqs:
            by_seg.setdefault(seg, []).append(sq)
        expected = lost = 0
        runs = {}
        for seg, sq in by_seg.items():
            sq.sort()
            span = sq[-1] - sq[0] + 1
            expected += span
            lost += span - len(set(sq))
            for n, c in loss_runs(sq).items():
                runs[n] = runs.get(n, 0) + c
        out['expected'] = expected
        out['lost'] = lost
        out['loss_pct'] = 100.0 * lost / expected if expected else 0.0
        out['runs'] = runs
        out['segments_seen'] = len(by_seg)
    lat.sort()
    ia.sort()
    out['lat'] = lat
    out['ia'] = ia
    return out


def pick_segment(segments, t):
    for s in segments:
        if s['t_lo'] <= t <= s['t_hi']:
            return s
    return segments[-1] if segments else None


def loss_runs(seqs):
    have = set(seqs)
    runs = {}
    lo, hi = seqs[0], seqs[-1]
    run = 0
    for s in range(lo, hi + 1):
        if s in have:
            if run:
                runs[run] = runs.get(run, 0) + 1
                run = 0
        else:
            run += 1
    if run:
        runs[run] = runs.get(run, 0) + 1
    return runs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--fps', type=float, default=FRAME_HINT_HZ,
                    help='nominal frame rate, for phrasing errors in frames')
    args = ap.parse_args()

    cams = sorted(int(m.group(1)) for m in
                  (re.match(r'syncfit_cam(\d+)\.csv$', f)
                   for f in os.listdir(args.run_dir)) if m)
    if not cams:
        print('no syncfit_cam*.csv in %s' % args.run_dir)
        return 1

    results = [analyse_cam(args.run_dir, c) for c in cams]
    frame_us = 1e6 / args.fps

    print('\n=== CLOCK FIT ===')
    print('%-4s %6s %6s %9s %9s %9s %9s' %
          ('cam', 'sync', 'used', 'cam_ppm', 'rms_us', 'worst_us', 'span_s'))
    for r in results:
        if not r['segments']:
            print('%-4d %6d   no usable fit' % (r['cam'], r['n_sync']))
            continue
        for i, s in enumerate(r['segments']):
            tag = '%d' % r['cam'] if i == 0 else '  +%d' % i
            span = (s['t_hi'] - s['t_lo']) / 1e6
            print('%-4s %6d %6d %9.2f %9.1f %9.1f %9.1f' %
                  (tag, s['n_total'], s['n_used'], -s['b'] * 1e6,
                   s['rms'], s['worst'], span))
    if any(len(r['segments']) > 1 for r in results):
        print('  (+N rows are post-reboot segments: the monotonic clock restarted)')
    print('  cam_ppm is the CAMERA crystal error vs the base (+ = camera fast).')
    print('  A cheap part is specified +/-30 ppm; well outside that is a bad board.')

    print('\n=== WHAT THE RESYNC BOUGHT ===')
    print('%-4s %14s %16s %12s' %
          ('cam', 'fitted_rms', 'start_only_error', 'in_frames'))
    for r in results:
        if not r['segments']:
            continue
        s = r['segments'][0]
        span = s['t_hi'] - s['t_lo']
        start_only = abs(s['b']) * span          # drift, uncorrected, over the run
        print('%-4d %11.2f ms %13.1f ms %12.1f' %
              (r['cam'], s['rms'] / 1000.0, start_only / 1000.0,
               start_only / frame_us))
    print('  start_only_error is what a single sync at t=0 would have accumulated')
    print('  by the end of the run. Compare it against one frame period (%.2f ms).'
          % (frame_us / 1000.0))

    print('\n=== PACKETS ===')
    print('%-4s %9s %9s %8s %8s %9s' %
          ('cam', 'rx', 'expected', 'lost', 'loss%', 'worst_run'))
    for r in results:
        if 'rx' not in r:
            continue
        worst = max(r['runs']) if r.get('runs') else 0
        print('%-4d %9d %9d %8d %8.3f %9d' %
              (r['cam'], r['rx'], r.get('expected', 0), r.get('lost', 0),
               r.get('loss_pct', 0.0), worst))
    for r in results:
        if r.get('runs'):
            top = sorted(r['runs'].items())[:6]
            print('  cam %d loss runs: %s' % (
                r['cam'], '  '.join('%dx%d' % (c, n) for n, c in top)))

    print('\n=== LATENCY (camera stamp -> base arrival) ===')
    print('%-4s %8s %8s %8s %8s %10s %10s' %
          ('cam', 'min_ms', 'p50_ms', 'p95_ms', 'p99_ms', 'spread_ms', 'ia_p99_ms'))
    for r in results:
        lat = r.get('lat') or []
        if not lat:
            continue
        p50, p95 = pct(lat, 0.5), pct(lat, 0.95)
        print('%-4d %8.2f %8.2f %8.2f %8.2f %10.2f %10.2f' %
              (r['cam'], lat[0] / 1000.0, p50 / 1000.0, p95 / 1000.0,
               pct(lat, 0.99) / 1000.0, (p95 - p50) / 1000.0,
               pct(r.get('ia') or [], 0.99) / 1000.0))
    print('  spread_ms (p95-p50) and ia_p99 are exact. The absolute columns carry')
    print('  the clock-fit uncertainty above - read them as +/- rms_us.')

    fits = [r for r in results if r['segments']]
    if len(fits) > 1:
        print('\n=== PAIRWISE SKEW ===')
        print('  Relative drift between cameras. This is what actually matters')
        print('  for multi-view: a common bias cancels in triangulation, a')
        print('  relative one does not.')
        print('%-9s %11s %14s' % ('pair', 'd_ppm', 'per_minute'))
        for i in range(len(fits)):
            for j in range(i + 1, len(fits)):
                db = -(fits[i]['segments'][0]['b'] - fits[j]['segments'][0]['b'])
                print('%-9s %11.2f %11.3f ms' %
                      ('%d-%d' % (fits[i]['cam'], fits[j]['cam']),
                       db * 1e6, abs(db) * 60e3))

    out_path = os.path.join(args.run_dir, 'clockfit.csv')
    with open(out_path, 'w') as f:
        f.write('# server_ref_us = a + (1+b) * cam_mono_us\n')
        f.write('# use b to remap; cam_ppm = -b*1e6 is the camera crystal error\n')
        f.write('cam,segment,a_us,b,cam_ppm,rms_us,n_used,t_lo_us,t_hi_us\n')
        for r in results:
            for i, s in enumerate(r['segments']):
                f.write('%d,%d,%.6f,%.12e,%.4f,%.3f,%d,%d,%d\n' %
                        (r['cam'], i, s['a'], s['b'], -s['b'] * 1e6,
                         s['rms'], s['n_used'], s['t_lo'], s['t_hi']))
    print('\nwrote %s' % out_path)
    tb = os.path.join(args.run_dir, 'timebase.txt')
    if os.path.exists(tb):
        print('ref -> Unix mapping in %s' % tb)
    return 0


if __name__ == '__main__':
    sys.exit(main())
