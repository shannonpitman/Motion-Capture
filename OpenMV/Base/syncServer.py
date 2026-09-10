#!/usr/bin/env python3
"""syncServer.py - Shannon Pitman

Base-station companion to timeSync.py on the cameras. Two jobs:

  1. Answer time-sync requests (UDP 7008) with the tightest t2/t3 this machine
     can produce.
  2. Sink the tracker's telemetry stream (UDP 7007), and account for every
     packet: drops, reordering, burst-loss run lengths, inter-arrival jitter
     and one-way latency, per camera.

Run it for the whole session, then feed its logs to fitClocks.py.

    python3 Base/syncServer.py --cams 1-6 --out ../runs/2026-09-10

WHY A SEPARATE THREAD FOR SYNC
t2 and t3 bracket the server's handling of one request. Anything that delays
them - a slow telemetry parse, a CSV flush, a console print - lands directly in
the camera's offset estimate as asymmetry. The responder therefore does nothing
but recvfrom/stamp/sendto, and the telemetry side never blocks it.

WHY THE REFERENCE CLOCK IS MONOTONIC, NOT time.time()
macOS's timed will step the wall clock while this runs. A step of even 50 ms
inside a session puts a fake discontinuity into every camera's drift fit and
quietly ruins it. So t2/t3 are monotonic microseconds with an arbitrary origin,
which is all the offset/drift maths needs. A single Unix timestamp rides along
in each reply purely so the cameras can set their RTCs for file naming, and the
ref->Unix mapping is written to the log header for later labelling.
"""

import argparse
import os
import socket
import struct
import sys
import threading
import time
from collections import deque

# -- protocol (must match timeSync.py) ---------------------------------------
VERSION = 1
REQ_MAGIC = b'SYNQ'
RSP_MAGIC = b'SYNR'
RPT_MAGIC = b'SYNP'
REQ_FMT = '<4sBBHQ'
RSP_FMT = '<4sBBHQQQQ'
RPT_FMT = '<4sBBHQQQQqI'
REQ_SIZE = struct.calcsize(REQ_FMT)
RSP_SIZE = struct.calcsize(RSP_FMT)
RPT_SIZE = struct.calcsize(RPT_FMT)

# -- protocol (must match kalmanFilter.py) -----------------------------------
PKT_MAGIC = 0x4B
HDR_FMT = '<BBBBHI'          # magic, cam_id, init_id, n_markers, seq, t_us
HDR_SIZE = struct.calcsize(HDR_FMT)
MARKER_FMT = '<fffB'         # u, v, sigma, flags
MARKER_SIZE = struct.calcsize(MARKER_FMT)

SEQ_MOD = 1 << 16            # seq is uint16 in the tracker's packet
TICK_MOD = 1 << 32           # t_us is uint32 on the wire

REORDER_WINDOW_S = 0.25      # how long a gap may still be filled by a late packet
REORDER_TICK_TOL_US = 1000000   # backward camera time up to this is a reorder
CLOCK_DISAGREE_US = 1000000     # camera vs base elapsed mismatch that means restart
LAT_WINDOW = 20000           # latency samples kept for percentiles


def now_ref_us():
    return time.monotonic_ns() // 1000


def now_unix_us():
    return time.time_ns() // 1000


def unwrap(raw, last_raw, hi, mod):
    """Extend a wrapping counter, tolerating small backward steps (reorders)."""
    if last_raw is None:
        return raw, raw, 0
    d = raw - last_raw
    if d < -(mod // 2):
        hi += mod
    elif d > (mod // 2):
        hi -= mod
    return hi + raw, raw, hi


def pct(sorted_vals, p):
    if not sorted_vals:
        return float('nan')
    k = (len(sorted_vals) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


class CamStats:
    """Per-camera accounting. Every counter here is answerable to one question:
    did this camera's data arrive, and when."""

    def __init__(self, cam_id, log_dir, log_markers=False):
        self.cam_id = cam_id
        self.first_ref = None
        self.last_ref = None

        self.received = 0
        self.lost = 0
        self.reordered = 0
        self.duplicated = 0
        self.bad = 0

        # A reorder looks exactly like a drop followed by a duplicate if you
        # only count sequence gaps. So a gap is held open for REORDER_WINDOW_S
        # and only charged as loss once nothing has filled it.
        self.missing = {}
        self.next_expect = None
        self.run_hist = {}           # consecutive-loss run length -> count

        self._seq_last = None
        self._seq_hi = 0
        self._tick_last = None
        self._tick_hi = 0
        self._max_tick_uw = None
        self._max_tick_ref = None
        self.seg = 0
        self.reboots = 0

        self.interarrival = deque(maxlen=LAT_WINDOW)
        self.latency = deque(maxlen=LAT_WINDOW)
        self._prev_ref = None

        self.init_id = None
        self.reacquires = 0
        self.offset = None           # camera mono us -> server ref us
        self.rtt = None
        self.sync_n = 0

        self.log_markers = log_markers
        path = os.path.join(log_dir, 'telem_cam%d.csv' % cam_id)
        self.f = open(path, 'w', buffering=1 << 20)
        cols = 'recv_ref_us,recv_unix_us,seg,init_id,seq,t_mono_us,n_markers'
        if log_markers:
            cols += ',markers'
        self.f.write('# ' + cols + '\n')

        # Two sync logs, because they answer different questions. syncraw is
        # every exchange the server answered - use it to see how much of a
        # burst is getting through. syncfit is only the samples the camera
        # ACCEPTED, and is what the drift fit consumes.
        self.sf = open(os.path.join(log_dir, 'syncraw_cam%d.csv' % cam_id),
                       'w', buffering=1)
        self.sf.write('# t1_mono_us,t2_ref_us,t3_ref_us\n')
        self.ff = open(os.path.join(log_dir, 'syncfit_cam%d.csv' % cam_id),
                       'w', buffering=1)
        self.ff.write('# n,t1_mono_us,t4_mono_us,t2_ref_us,t3_ref_us,'
                      'delay_us,offset_us\n')

    # -- telemetry --------------------------------------------------------
    def on_packet(self, data, ref_us, unix_us):
        if len(data) < HDR_SIZE:
            self.bad += 1
            return
        magic, cam_id, init_id, n_mark, seq_raw, tick_raw = \
            struct.unpack_from(HDR_FMT, data, 0)
        if magic != PKT_MAGIC:
            self.bad += 1
            return
        if len(data) != HDR_SIZE + n_mark * MARKER_SIZE:
            self.bad += 1
            return

        self.received += 1
        if self.first_ref is None:
            self.first_ref = ref_us
        self.last_ref = ref_us

        if self.init_id is not None and init_id != self.init_id:
            self.reacquires += 1
        self.init_id = init_id

        seq, self._seq_last, self._seq_hi = unwrap(
            seq_raw, self._seq_last, self._seq_hi, SEQ_MOD)
        tick, self._tick_last, self._tick_hi = unwrap(
            tick_raw, self._tick_last, self._tick_hi, TICK_MOD)

        # A camera reboot and a genuine seq wrap look IDENTICAL in the sequence
        # number alone - at 200 fps seq wraps every 5.5 minutes, and either way
        # it jumps backwards by tens of thousands. Charging that as loss would
        # invent ~25000 dropped packets. The camera clock is the disambiguator:
        # across a real wrap it keeps running, across a reboot it restarts at
        # zero.
        #
        # Compare against the HIGHEST camera time seen, not the last one, and
        # allow a second of backward movement. A reordered packet also arrives
        # with a backward timestamp, and treating that as a reboot would wipe
        # the gap bookkeeping and silently discard the very losses being
        # measured. Reorder is sub-second; a reboot throws away the whole
        # uptime, and if it does not, the second test catches it because the
        # reboot gap itself puts camera and base elapsed time far apart.
        if self._max_tick_uw is not None:
            d_cam = tick - self._max_tick_uw
            d_base = ref_us - self._max_tick_ref
            if d_cam < -REORDER_TICK_TOL_US or \
               abs(d_cam - d_base) > CLOCK_DISAGREE_US:
                self._restart()
                seq, self._seq_last, self._seq_hi = unwrap(
                    seq_raw, None, 0, SEQ_MOD)
                tick, self._tick_last, self._tick_hi = unwrap(
                    tick_raw, None, 0, TICK_MOD)
                self._max_tick_uw = None
        if self._max_tick_uw is None or tick > self._max_tick_uw:
            self._max_tick_uw = tick
            self._max_tick_ref = ref_us

        self._account(seq, ref_us)

        if self._prev_ref is not None:
            self.interarrival.append(ref_us - self._prev_ref)
        self._prev_ref = ref_us

        # Live latency is only as good as the last offset; the honest number
        # comes from fitClocks.py once the whole drift curve is known.
        if self.offset is not None:
            self.latency.append(ref_us - (tick + self.offset))

        row = '%d,%d,%d,%d,%d,%d,%d' % (ref_us, unix_us, self.seg, init_id,
                                        seq, tick, n_mark)
        if self.log_markers:
            vals = []
            for i in range(n_mark):
                u, v, s, fl = struct.unpack_from(
                    MARKER_FMT, data, HDR_SIZE + i * MARKER_SIZE)
                vals.append('%.3f %.3f %.4f %d' % (u, v, s, fl))
            row += ',' + ';'.join(vals)
        self.f.write(row + '\n')

    def _restart(self):
        # Discard the in-flight gap bookkeeping: sequence numbers either side
        # of a restart are not comparable, so holding those gaps open would
        # charge them as loss the moment they aged out.
        self.reboots += 1
        self.seg += 1
        self.missing = {}
        self.next_expect = None
        self.offset = None      # camera mono restarted; old offset is meaningless
        self._prev_ref = None
        print('cam %d: clock restart detected (segment %d)' % (self.cam_id, self.seg))

    def _account(self, seq, ref_us):
        if self.next_expect is None:
            self.next_expect = seq + 1
            return
        if seq == self.next_expect:
            self.next_expect += 1
        elif seq > self.next_expect:
            deadline = ref_us + REORDER_WINDOW_S * 1e6
            for m in range(self.next_expect, seq):
                self.missing[m] = deadline
            self.next_expect = seq + 1
        else:
            if self.missing.pop(seq, None) is not None:
                self.reordered += 1
            else:
                self.duplicated += 1

    def reap(self, ref_us):
        """Charge gaps that nobody filled, grouping contiguous ones into runs.
        Run length is what matters to a tracker: ten scattered singles are
        survivable, one run of ten is a quarter-second hole."""
        due = sorted(m for m, d in self.missing.items() if d <= ref_us)
        if not due:
            return
        run = 1
        for i in range(1, len(due) + 1):
            if i < len(due) and due[i] == due[i - 1] + 1:
                run += 1
            else:
                self.run_hist[run] = self.run_hist.get(run, 0) + 1
                run = 1
        for m in due:
            del self.missing[m]
        self.lost += len(due)

    # -- sync -------------------------------------------------------------
    def on_report(self, n, t1, t4, t2, t3, offset, delay):
        self.offset = offset
        self.rtt = delay
        self.sync_n += 1
        self.ff.write('%d,%d,%d,%d,%d,%d,%d\n' % (n, t1, t4, t2, t3, delay, offset))

    # -- reporting --------------------------------------------------------
    def summary(self, ref_us):
        sent = self.received + self.lost
        loss = 100.0 * self.lost / sent if sent else 0.0
        span = (self.last_ref - self.first_ref) / 1e6 if self.first_ref else 0.0
        fps = (self.received - 1) / span if span > 0 else 0.0
        lat = sorted(self.latency)
        ia = sorted(self.interarrival)
        silent = (ref_us - self.last_ref) / 1e6 if self.last_ref else float('inf')
        return dict(cam=self.cam_id, rx=self.received, lost=self.lost,
                    loss_pct=loss, reord=self.reordered, dup=self.duplicated,
                    bad=self.bad, fps=fps, silent_s=silent,
                    lat_med=pct(lat, 0.5) / 1000.0, lat_p95=pct(lat, 0.95) / 1000.0,
                    ia_med=pct(ia, 0.5) / 1000.0, ia_p99=pct(ia, 0.99) / 1000.0,
                    rtt=self.rtt / 1000.0 if self.rtt else float('nan'),
                    sync_n=self.sync_n, reacq=self.reacquires, reboots=self.reboots,
                    worst_run=max(self.run_hist) if self.run_hist else 0)

    def close(self):
        self.f.close()
        self.sf.close()
        self.ff.close()


class Server:
    def __init__(self, cams, telem_port, sync_port, out_dir, log_markers):
        self.expected = set(cams)
        self.out_dir = out_dir
        self.log_markers = log_markers
        self.stats = {}
        self.lock = threading.Lock()
        self.running = True
        self.unexpected = set()

        self.ref0 = now_ref_us()
        self.unix0 = now_unix_us()
        with open(os.path.join(out_dir, 'timebase.txt'), 'w') as f:
            f.write('# monotonic reference -> Unix, captured at server start\n')
            f.write('ref0_us=%d\nunix0_us=%d\nhost=%s\n'
                    % (self.ref0, self.unix0, socket.gethostname()))

        self.sync_sock = self._bind(sync_port)
        self.telem_sock = self._bind(telem_port, rcvbuf=4 << 20)

    def _bind(self, port, rcvbuf=None):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if rcvbuf:
            # A big receive buffer is the cheapest drop prevention there is: it
            # absorbs a GC pause or a scheduler hiccup that would otherwise
            # overflow the socket while this process is not running.
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
            except OSError:
                pass
        s.bind(('0.0.0.0', port))
        s.settimeout(0.5)
        return s

    def cam(self, cam_id):
        st = self.stats.get(cam_id)
        if st is None:
            if cam_id not in self.expected and cam_id not in self.unexpected:
                self.unexpected.add(cam_id)
                print('WARNING: traffic from unexpected cam_id %d' % cam_id)
            st = CamStats(cam_id, self.out_dir, self.log_markers)
            self.stats[cam_id] = st
            print('cam %d: first packet' % cam_id)
        return st

    # -- threads ----------------------------------------------------------
    def sync_loop(self):
        rsp = bytearray(RSP_SIZE)
        while self.running:
            try:
                data, addr = self.sync_sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            t2 = now_ref_us()
            if len(data) == RPT_SIZE and data[:4] == RPT_MAGIC:
                (_, ver, cam_id, n, t1, t4, rt2, rt3, offset, delay) = \
                    struct.unpack(RPT_FMT, data)
                if ver == VERSION:
                    with self.lock:
                        self.cam(cam_id).on_report(n, t1, t4, rt2, rt3,
                                                   offset, delay)
                continue
            if len(data) != REQ_SIZE:
                continue
            magic, ver, cam_id, seq, t1 = struct.unpack(REQ_FMT, data)
            if magic != REQ_MAGIC or ver != VERSION:
                continue
            unix = now_unix_us()
            t3 = now_ref_us()
            struct.pack_into(RSP_FMT, rsp, 0, RSP_MAGIC, VERSION, cam_id,
                             seq, t1, t2, t3, unix)
            try:
                self.sync_sock.sendto(rsp, addr)
            except OSError:
                continue
            # The camera picks the best sample of the burst; we log every
            # exchange so the fit can see what was discarded and why.
            with self.lock:
                st = self.cam(cam_id)
                st.sf.write('%d,%d,%d,,,\n' % (t1, t2, t3))

    def telem_loop(self):
        last_reap = now_ref_us()
        while self.running:
            try:
                data, addr = self.telem_sock.recvfrom(2048)
            except socket.timeout:
                data = None
            except OSError:
                break
            ref = now_ref_us()
            if data:
                unix = now_unix_us()
                if len(data) >= 2 and data[0] == PKT_MAGIC:
                    with self.lock:
                        self.cam(data[1]).on_packet(data, ref, unix)
            if ref - last_reap > 100000:
                with self.lock:
                    for st in self.stats.values():
                        st.reap(ref)
                last_reap = ref

    def report(self):
        ref = now_ref_us()
        with self.lock:
            rows = [self.stats[c].summary(ref) for c in sorted(self.stats)]
            missing = sorted(self.expected - set(self.stats))
        print('\n%-4s %8s %6s %7s %6s %5s %7s %7s %7s %7s %6s %5s'
              % ('cam', 'rx', 'lost', 'loss%', 'run', 'reord',
                 'fps', 'lat_ms', 'p95_ms', 'ia_p99', 'rtt', 'sync'))
        for r in rows:
            flag = '  SILENT %.0fs' % r['silent_s'] if r['silent_s'] > 3 else ''
            if r['reacq']:
                flag += '  reacq=%d' % r['reacq']
            if r['reboots']:
                flag += '  RESTARTS=%d' % r['reboots']
            print('%-4d %8d %6d %7.3f %6d %5d %7.1f %7.2f %7.2f %7.2f %6.1f %5d%s'
                  % (r['cam'], r['rx'], r['lost'], r['loss_pct'], r['worst_run'],
                     r['reord'], r['fps'], r['lat_med'], r['lat_p95'],
                     r['ia_p99'], r['rtt'], r['sync_n'], flag))
        if missing:
            print('NO TRAFFIC from expected cams: %s'
                  % ', '.join(str(c) for c in missing))

    def close(self):
        self.running = False
        with self.lock:
            for st in self.stats.values():
                st.close()
        self.sync_sock.close()
        self.telem_sock.close()


def parse_cams(spec):
    out = []
    for part in spec.split(','):
        part = part.strip()
        if '-' in part:
            a, b = part.split('-')
            out.extend(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cams', default='1-6',
                    help='expected camera ids, e.g. "1-6" or "1,2,5" (default 1-6)')
    ap.add_argument('--telem-port', type=int, default=7007)
    ap.add_argument('--sync-port', type=int, default=7008)
    ap.add_argument('--out', default='runs/run')
    ap.add_argument('--status', type=float, default=5.0,
                    help='seconds between console tables (0 = silent)')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='stop after N seconds (0 = until Ctrl-C)')
    ap.add_argument('--log-markers', action='store_true',
                    help='also log centroids. Off by default: at 6 cams x 200 fps '
                         'this file grows ~0.5 GB/hour and the drop/latency '
                         'analysis does not need it.')
    args = ap.parse_args()

    cams = parse_cams(args.cams)
    os.makedirs(args.out, exist_ok=True)
    srv = Server(cams, args.telem_port, args.sync_port, args.out, args.log_markers)

    print('sync   responder on udp/%d' % args.sync_port)
    print('telem  sink      on udp/%d' % args.telem_port)
    print('expecting cams: %s' % ', '.join(str(c) for c in cams))
    print('logging to %s' % os.path.abspath(args.out))

    threads = [threading.Thread(target=srv.sync_loop, daemon=True),
               threading.Thread(target=srv.telem_loop, daemon=True)]
    for t in threads:
        t.start()

    t0 = time.monotonic()
    try:
        while True:
            time.sleep(args.status if args.status > 0 else 1.0)
            if args.status > 0:
                srv.report()
            if args.duration and (time.monotonic() - t0) >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    print('\n=== final ===')
    srv.report()
    srv.close()
    print('\nNow run:  python3 Base/fitClocks.py %s' % args.out)


if __name__ == '__main__':
    sys.exit(main())
