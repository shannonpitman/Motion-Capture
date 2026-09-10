#!/usr/bin/env python3
"""simCameras.py - Shannon Pitman

Fake N cameras against a live syncServer.py, so the base-station pipeline can
be shaken down before six real cameras exist.

    python3 Base/syncServer.py --cams 1-6 --out /tmp/simrun --status 5
    python3 Base/simCameras.py --cams 6 --fps 200 --loss 0.004 --seconds 90
    python3 Base/fitClocks.py /tmp/simrun --fps 200

Each simulated camera runs in its own process and imports the REAL timeSync.py
under a small MicroPython shim (ticks_us/ticks_diff/sleep_ms), so what is being
tested is the firmware module that will ship, not a reimplementation of it.
Each is given its own crystal error, so fitClocks.py can be checked against
drifts whose true values are known.

The shim's ticks_us wraps at 2^30 like the real one. --tick-mod shrinks that
so a wrap can be forced in a 60 s test instead of waiting 17.9 minutes.
"""

import argparse
import os
import random
import struct
import subprocess
import sys
import time as _rt

PKT_MAGIC = 0x4B

# Crystal errors, ppm. Deliberately spread across the +/-30 ppm a cheap part
# is specified for, and deliberately of both signs so pairwise skew is not
# accidentally near zero.
DEFAULT_PPM = [37.0, -12.5, 4.0, -28.0, 19.5, -6.0]


def run_camera(cam_id, host, fps, loss, seconds, ppm, tick_mod, n_markers, seed,
               sync_period_ms):
    import time as time_mod
    rng = random.Random(seed)

    t0 = _rt.perf_counter()
    rate = 1.0 + ppm * 1e-6
    boot = rng.randrange(0, 1 << 20)     # cameras did not boot together

    def ticks_us():
        return int((_rt.perf_counter() - t0) * 1e6 * rate + boot) % tick_mod

    def ticks_diff(a, b):
        d = (a - b) & (tick_mod - 1)
        return d - tick_mod if d > tick_mod // 2 else d

    def ticks_ms():
        return ticks_us() // 1000

    def ticks_add(t, d):
        return (t + d) % tick_mod

    time_mod.ticks_us = ticks_us
    time_mod.ticks_diff = ticks_diff
    time_mod.ticks_ms = ticks_ms
    time_mod.ticks_add = ticks_add
    time_mod.sleep_ms = lambda ms: _rt.sleep(ms / 1000.0)

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import timeSync

    ts = timeSync.TimeSync(cam_id, host, period_ms=sync_period_ms, log=False)
    ts.startup_sync(set_rtc=False, timeout_ms=4000)

    import socket
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (host, 7007)
    fmt = '<BBBBHI' + 'fffB' * n_markers
    pkt = bytearray(struct.calcsize(fmt))

    period = 1.0 / fps
    seq = 0
    next_t = _rt.perf_counter()
    end = _rt.perf_counter() + seconds
    sent = dropped = 0
    while _rt.perf_counter() < end:
        now = _rt.perf_counter()
        if now < next_t:
            _rt.sleep(min(next_t - now, 0.002))
            continue
        next_t += period
        t_us = ts.poll()
        vals = [PKT_MAGIC, cam_id, 0, n_markers, seq & 0xFFFF, t_us & 0xFFFFFFFF]
        for i in range(n_markers):
            vals += [800.0 + i, 600.0 + i, 0.05, 1]
        struct.pack_into(fmt, pkt, 0, *vals)
        seq += 1
        # Loss is injected in bursts, not independently per packet: that is how
        # WiFi actually fails, and a run of ten is a very different problem to
        # ten scattered singles.
        if rng.random() < loss:
            burst = rng.choice([1, 1, 1, 2, 3, 7])
            dropped += burst
            for _ in range(burst - 1):
                seq += 1
                next_t += period
            continue
        try:
            tx.sendto(pkt, target)
            sent += 1
        except OSError:
            pass
    ts.close()
    print('cam %d: sent=%d dropped=%d ppm=%+.1f offset=%s'
          % (cam_id, sent, dropped, ppm,
             ts.last_offset if ts.last_offset is not None else 'NONE'))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cams', type=int, default=6)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--fps', type=float, default=200.0)
    ap.add_argument('--loss', type=float, default=0.004,
                    help='probability per packet of starting a loss burst')
    ap.add_argument('--seconds', type=float, default=90.0)
    ap.add_argument('--markers', type=int, default=1)
    ap.add_argument('--tick-mod', type=int, default=1 << 30)
    ap.add_argument('--ppm', default='',
                    help='comma-separated crystal errors, one per camera')
    ap.add_argument('--_child', type=int, default=0)
    ap.add_argument('--_ppm', type=float, default=0.0)
    ap.add_argument('--sync-period', type=int, default=20000,
                    help='steady-state resync cadence, ms')
    args = ap.parse_args()

    if args._child:
        run_camera(args._child, args.host, args.fps, args.loss, args.seconds,
                   args._ppm, args.tick_mod, args.markers, args._child * 7919,
                   args.sync_period)
        return 0

    ppms = [float(x) for x in args.ppm.split(',')] if args.ppm else DEFAULT_PPM
    procs = []
    print('true drifts: %s' % ', '.join(
        'cam%d=%+.1f ppm' % (i + 1, ppms[i % len(ppms)]) for i in range(args.cams)))
    for i in range(args.cams):
        cmd = [sys.executable, os.path.abspath(__file__),
               '--_child', str(i + 1), '--_ppm', str(ppms[i % len(ppms)]),
               '--host', args.host, '--fps', str(args.fps),
               '--loss', str(args.loss), '--seconds', str(args.seconds),
               '--markers', str(args.markers), '--tick-mod', str(args.tick_mod),
               '--sync-period', str(args.sync_period)]
        procs.append(subprocess.Popen(cmd))
        _rt.sleep(0.15)
    for p in procs:
        p.wait()
    return 0


if __name__ == '__main__':
    sys.exit(main())
