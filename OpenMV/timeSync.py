# timeSync.py - Shannon Pitman
# Wall-clock sync for OpenMV RT1062 cameras over the existing WiFi link.
#
# Import into the tracker; one call per frame does everything:
#
#     import timeSync
#     ts = timeSync.TimeSync(CAM_ID, HOST_IP)
#     ts.startup_sync(set_rtc=True)      # blocking, ONCE, before the trial
#     ...
#     while True:
#         t_us = ts.poll()               # non-blocking: advances clock, services sync
#         ...
#         send(seq, t_us, tracks)
#
# WHAT THIS IS FOR, AND WHAT IT IS NOT FOR
# This gives every camera a COMMON TIME ORIGIN for labelling, file naming and
# post-hoc alignment. Over WiFi it is good to a few ms, no better - the
# round-trip is asymmetric and SNTP-style maths charges you half the asymmetry.
# It does NOT replace the strobe. Strobe-edge row reading is ~21 us (NOTES.md,
# "The rolling shutter is also the solution"); this is ~1-3 ms after a drift
# fit. Three orders of magnitude apart. Use this to label, the strobe to align.
#
# THE CLOCK IS NEVER STEPPED DURING A TRIAL.
# Stepping puts a discontinuity in the middle of the data that no filter can
# undo. Samples are logged raw and the base fits offset + drift afterwards
# (Base/fitClocks.py). The only step is the optional RTC set at startup, which
# happens before the first frame.
#
# WHY mono_us() EXISTS
# time.ticks_us() wraps at 2^30 us = 17.9 minutes. ticks_diff() handles that
# correctly, so per-stage timing reports are fine, but ABSOLUTE stamps fold
# back on themselves partway through a long trial. mono_us() accumulates
# ticks_diff into an unbounded int, so it only works if it is called more often
# than every ~8 minutes. At tracking rates that is every frame. It is NOT safe
# to leave the tracker paused for 9 minutes and then resume.

import time
import struct
import socket

VERSION = 1

SYNC_PORT = 7008          # base-side responder. NOT 123 - see notes below.
CAM_PORT_BASE = 7100      # this camera binds CAM_PORT_BASE + cam_id

# Deliberately not UDP/123. Binding 123 on the base needs sudo, macOS's own
# timed may hold it, and NTP's protocol baggage buys nothing when you control
# both ends. Same maths (Cristian's algorithm), no root, and we get the
# round-trip time back so bad samples can be thrown away.

REQ_MAGIC = b'SYNQ'
RSP_MAGIC = b'SYNR'
RPT_MAGIC = b'SYNP'
REQ_FMT = '<4sBBHQ'        # magic, ver, cam_id, seq, t1 (camera mono us)
RSP_FMT = '<4sBBHQQQQ'     # + t2_ref, t3_ref (server mono us), t3_unix (us)
RPT_FMT = '<4sBBHQQQQqI'   # accepted sample: t1, t4, t2, t3, offset, delay
REQ_SIZE = struct.calcsize(REQ_FMT)
RSP_SIZE = struct.calcsize(RSP_FMT)
RPT_SIZE = struct.calcsize(RPT_FMT)

# The camera picks the best sample of each burst, and only the camera knows t4.
# Reporting the accepted sample back means the base can do the whole drift fit
# without anyone pulling an SD card after a session. The SD log stays as the
# backup for when the link is what failed.

# Burst: send N requests, keep the one with the LOWEST round-trip delay. The
# minimum-delay sample is the one whose path was least asymmetric, so it has
# the smallest offset error. Averaging the burst instead would fold every
# retry-inflated sample back in and is measurably worse on WiFi.
BURST_N = 8
BURST_SPACING_MS = 40
BURST_TIMEOUT_MS = 400
PERIOD_MS = 20000          # steady-state cadence between bursts
WARMUP_PERIOD_MS = 2000    # faster cadence until the fit has something to chew
WARMUP_SAMPLES = 12
MAX_DELAY_US = 60000       # discard samples worse than this round trip

# WHY THE WARMUP EXISTS
# Drift is a SLOPE, and the slope's uncertainty is roughly
#     sigma_b ~ sigma_offset * sqrt(12) / (span * sqrt(n))
# With WiFi asymmetry putting sigma_offset near 0.5 ms, four samples over a
# minute is +/-50 ppm - worse than the +/-30 ppm crystal error being measured,
# so the fit actively makes things worse. Twelve samples in the first 25 s
# brings that to a few ppm immediately, and after that 20 s is plenty because
# span is growing on its own. A short trial is exactly the case where the slow
# cadence alone would have produced a confident, wrong drift.

LOG_PATH = '/sdcard/sync_cam%d.csv'
FLUSH_EVERY = 16           # rows buffered before touching the SD card

_IDLE, _BURST = 0, 1

# MicroPython's epoch is 2000-01-01 on bare metal, 1970-01-01 on unix builds.
# Detect it rather than assume - getting this wrong shifts the RTC by 30 years.
_EPOCH_OFF = 946684800 if time.gmtime(0)[0] == 2000 else 0


class TimeSync:
    def __init__(self, cam_id, host_ip, sync_port=SYNC_PORT, period_ms=PERIOD_MS,
                 log=True):
        self.cam_id = cam_id
        self.target = (host_ip, sync_port)
        self.period_ms = period_ms

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)   # never let a stalled link stall the tracker
        try:
            self.sock.bind(('0.0.0.0', CAM_PORT_BASE + cam_id))
        except OSError:
            pass  # port busy after a soft reset; an ephemeral port still works

        # Monotonic clock state. Unbounded int, immune to the 2^30 us wrap.
        self._last_ticks = time.ticks_us()
        self._mono = 0

        self._state = _IDLE
        self._burst_seq = 0
        self._sent = 0
        self._t_next_send = 0
        self._t_burst_start = 0
        self._pending = {}             # seq -> t1
        self._best = None              # (delay, offset, t1, t2, t3, t3_unix)

        self.samples = 0               # accepted bursts
        self.last_offset = None        # camera mono us -> server ref us
        self.last_delay = None
        self.last_sync_mono = None
        self.rejected = 0

        self._req = bytearray(REQ_SIZE)
        self._rpt = bytearray(RPT_SIZE)
        self._rows = []
        self._log = log
        self._logf = None
        if log:
            self._open_log()

    # -- clock ------------------------------------------------------------
    def mono_us(self):
        # Unbounded monotonic microseconds. Must be called at least every
        # ~8 minutes or the underlying ticks_us wrap is missed.
        now = time.ticks_us()
        self._mono += time.ticks_diff(now, self._last_ticks)
        self._last_ticks = now
        return self._mono

    def to_server(self, mono_us):
        # Best-effort live conversion. The AUTHORITATIVE mapping is the drift
        # fit done on the base afterwards; this is for on-camera printouts.
        if self.last_offset is None:
            return None
        return mono_us + self.last_offset

    # -- main entry point -------------------------------------------------
    def poll(self):
        # Call once per frame. Advances the monotonic clock, services at most
        # one send and drains pending replies, returns monotonic us.
        # Outside a burst window this is a ticks_us() call and two compares.
        t = self.mono_us()
        if self._state == _IDLE:
            period = (WARMUP_PERIOD_MS if self.samples < WARMUP_SAMPLES
                      else self.period_ms)
            if self.last_sync_mono is None or \
               (t - self.last_sync_mono) >= period * 1000:
                self._begin_burst(t)
        else:
            self._service_burst(t)
        return t

    # -- burst state machine ----------------------------------------------
    def _begin_burst(self, t):
        self._drain()                  # discard stale replies from last burst
        self._state = _BURST
        self._sent = 0
        self._t_next_send = t
        self._t_burst_start = t
        self._pending = {}
        self._best = None

    def _service_burst(self, t):
        self._drain()
        if self._sent < BURST_N and t >= self._t_next_send:
            self._send_one(t)
            self._t_next_send = t + BURST_SPACING_MS * 1000
            return
        done = self._sent >= BURST_N and \
            (t - self._t_next_send) >= BURST_TIMEOUT_MS * 1000
        if done or (t - self._t_burst_start) > 5000000:
            self._end_burst(t)

    def _send_one(self, t):
        self._burst_seq = (self._burst_seq + 1) & 0xFFFF
        seq = self._burst_seq
        # t1 is taken as late as possible: pack first, stamp, then send.
        struct.pack_into(REQ_FMT, self._req, 0, REQ_MAGIC, VERSION,
                         self.cam_id, seq, 0)
        t1 = self.mono_us()
        struct.pack_into('<Q', self._req, REQ_SIZE - 8, t1)
        self._pending[seq] = t1
        try:
            self.sock.sendto(self._req, self.target)
            self._sent += 1
        except OSError:
            self._sent += 1            # link down; the burst will just time out

    def _drain(self):
        while True:
            try:
                msg = self.sock.recv(RSP_SIZE)
            except OSError:
                return
            if msg is None or len(msg) != RSP_SIZE:
                continue
            t4 = self.mono_us()
            magic, ver, cid, seq, t1e, t2, t3, t3u = struct.unpack(RSP_FMT, msg)
            if magic != RSP_MAGIC or ver != VERSION or cid != self.cam_id:
                continue
            t1 = self._pending.pop(seq, None)
            if t1 is None or t1 != t1e:
                continue               # stale or forged; ignore
            delay = (t4 - t1) - (t3 - t2)
            if delay < 0 or delay > MAX_DELAY_US:
                self.rejected += 1
                continue
            # Cristian's algorithm. offset maps camera mono -> server ref.
            offset = ((t2 - t1) + (t3 - t4)) // 2
            if self._best is None or delay < self._best[0]:
                self._best = (delay, offset, t1, t2, t3, t3u, t4)

    def _end_burst(self, t):
        self._state = _IDLE
        self.last_sync_mono = t
        if self._best is None:
            return
        delay, offset, t1, t2, t3, t3u, t4 = self._best
        self.samples += 1
        self.last_offset = offset
        self.last_delay = delay
        # Row is deliberately RAW. No smoothing, no stepping - the fit happens
        # on the base where the whole series is visible at once.
        self._rows.append((self.samples, t1, t4, t2, t3, t3u, delay, offset))
        if len(self._rows) >= FLUSH_EVERY:
            self.flush()
        self._report(t1, t4, t2, t3, offset, delay)

    def _report(self, t1, t4, t2, t3, offset, delay):
        # Best effort. If it is lost the SD log still has the row, and one
        # missing sample out of a 20 s cadence costs the fit nothing.
        try:
            struct.pack_into(RPT_FMT, self._rpt, 0, RPT_MAGIC, VERSION,
                             self.cam_id, self.samples & 0xFFFF,
                             t1, t4, t2, t3, offset, delay)
            self.sock.sendto(self._rpt, self.target)
        except (OSError, OverflowError):
            pass

    # -- one-shot startup sync --------------------------------------------
    def startup_sync(self, set_rtc=True, timeout_ms=8000):
        # Blocking, and only safe BEFORE the tracking loop starts. Returns the
        # accepted offset, or None. Optionally steps the RTC - the single
        # deliberate step, taken before the first frame is ever captured.
        t0 = self.mono_us()
        self._begin_burst(t0)
        while self._state == _BURST:
            t = self.mono_us()
            if (t - t0) > timeout_ms * 1000:
                self._end_burst(t)
                break
            self._service_burst(t)
            time.sleep_ms(2)
        if self.last_offset is None:
            print("timeSync: no reply from %s:%d - running unsynced" % self.target)
            return None
        print("timeSync: offset=%d us  rtt=%d us  (%d/%d replies)"
              % (self.last_offset, self.last_delay,
                 BURST_N - len(self._pending), BURST_N))
        if set_rtc:
            self.set_rtc()
        return self.last_offset

    def set_rtc(self):
        # With a coin cell on the VBAT pads the RTC survives a power cycle, so
        # a camera that reboots mid-session comes back with usable wall time
        # and can name its files sensibly before the first resync lands.
        if self._best is None:
            return False
        t3u = self._best[5]
        unix_s = t3u // 1000000
        sub_us = t3u % 1000000
        gm = time.gmtime(unix_s - _EPOCH_OFF)
        # (year, month, mday, hour, min, sec, weekday[0=Mon], yearday)
        dt = (gm[0], gm[1], gm[2], gm[6] + 1, gm[3], gm[4], gm[5], sub_us)
        try:
            import pyb
            pyb.RTC().datetime(dt)
        except Exception:
            try:
                from machine import RTC
                RTC().datetime(dt)
            except Exception as e:
                print("timeSync: RTC set failed: %s" % e)
                return False
        # VERIFY THE FIELD ORDER ON YOUR FIRMWARE. pyb.RTC and machine.RTC
        # disagree across ports on both the weekday convention and whether the
        # last field is subseconds or microseconds. Print it once and check.
        print("timeSync: RTC set to %04d-%02d-%02d %02d:%02d:%02d UTC"
              % (gm[0], gm[1], gm[2], gm[3], gm[4], gm[5]))
        return True

    # -- logging ----------------------------------------------------------
    def _open_log(self):
        try:
            self._logf = open(LOG_PATH % self.cam_id, 'a')
            self._logf.write('# n,t1_mono_us,t4_mono_us,t2_ref_us,t3_ref_us,'
                             't3_unix_us,delay_us,offset_us\n')
            self._logf.flush()
        except OSError as e:
            print("timeSync: no SD log (%s)" % e)
            self._logf = None
            self._log = False

    def flush(self):
        # Rows are buffered because an SD block erase can stall for tens of ms
        # and that must never land inside the timestamp path. Bursts are 20 s
        # apart so FLUSH_EVERY=16 touches the card about every 5 minutes.
        if not self._logf or not self._rows:
            return
        try:
            for r in self._rows:
                self._logf.write('%d,%d,%d,%d,%d,%d,%d,%d\n' % r)
            self._logf.flush()
            self._rows = []
        except OSError as e:
            print("timeSync: log write failed: %s" % e)

    def close(self):
        self.flush()
        if self._logf:
            self._logf.close()
            self._logf = None

    def status(self):
        if self.last_offset is None:
            return "timeSync: unsynced"
        return "timeSync: n=%d offset=%d us rtt=%d us rejected=%d" % (
            self.samples, self.last_offset, self.last_delay, self.rejected)
