# extrinsicsCalib.py - extrinsic calibration capture for easyWand6
# Shannon Pitman
#
# Runs on each OpenMV RT1062. Captures strobed IR marker centroids in SENSOR
# coordinates and streams them to MATLAB over UDP. Standalone - shares no
# imports with kalmanFilter.py, though the geometry and centroid maths are
# lifted from it unchanged so the two agree on what a sensor pixel is.
#
# Deliberate differences from kalmanFilter.py:
#   - No Kalman filter, no association, no windowing. Every frame is a full
#     field scan, so t_readout is CONSTANT and the strobe hit rate is
#     predictable. Windowing would make both vary frame to frame.
#   - Marker identity comes from rig geometry (order_markers), not temporal
#     tracking, so every camera labels A/B/C/D identically with no
#     communication between them. This is what easyWand needs: within one
#     row, pt1 must be the same physical marker in every camera.
#   - Long exposure. The strobe defines the instant; the exposure only has to
#     be open when the pulse arrives.
#
# Cost: full field at VGA framesize means ratio ~4, so centroids are ~4x
# coarser in sensor px than the windowed tracker achieves. That is the correct
# trade - calibration needs markers in the image corners far more than it
# needs precision, and 0.2-0.6 sensor px is in line with the easyWand test
# cases (0.15-1.8 px).
#
# SET CAM_ID PER CAMERA BEFORE FLASHING. Everything else is identical.
#
# MODE = "expose"  -> exposure stays long; turn the LED trimpot until peak
#                     reads 200-230 with sat=0 and blobs=N_MARKERS.
# MODE = "readout" -> measures t_readout from the strobe band height.
# MODE = "run"     -> streams packets to MATLAB.

import csi
import time
import math
import gc
import struct

MODE = "expose"          # "expose" | "readout" | "run"

CAM_ID = 1               # 1..7  <-- CHANGE PER CAMERA
N_MARKERS = 4            # A, B, C, D. Must be 4 - ordering needs all four.

ENABLE_UDP = True
WIFI_SSID = "msg.network"
WIFI_KEY = "msg.t66Yu9"
HOST_IP = "192.168.0.32"
HOST_PORT = 7008         # 7007 is the tracker. Keep them separate.

# Capture -------------------------------------------------------------------
# EXPOSURE_US must exceed t_readout or the strobe lights only a band of rows.
# Measure t_readout with MODE="readout", then set this above it.
EXPOSURE_US = 30000
GAIN_DB = 0
FRAMESIZE = csi.VGA      # full sensor field. QVGA halves t_readout if needed.

# Detection -----------------------------------------------------------------
THRESH_LO = 80           # re-tune: long exposure raises the ambient floor
REFINE_FLOOR = 50        # re-run kalmanFilter.py MODE="floor" after the
CENTROID_SIGMA = 0.0544  # exposure change - all three come from one row
AREA_NOMINAL_SENSOR = 980.0

AREA_MIN_SENSOR = 120.0
MIN_AREA_IMG = 4
REFINE_PAD = 2
REFINE_MAX_PX = 3000
SAT_LEVEL = 250
SAT_R_INFLATE = 6.0
SIGMA_MIN = 0.03
SIGMA_MAX = 10.0

# Ordering ------------------------------------------------------------------
# Rig: A --- B ------ C collinear, D off the line.
# B sits off-centre so |AB|/|BC| != 1 and A is the end FARTHER from B.
# Collinearity and betweenness survive perspective exactly, so D and B are
# rigorous. The A/C call uses |AB|/|BC|, affine-invariant only - but a 2:1
# design ratio gives a 4x margin between the two hypotheses, which tolerates
# heavy foreshortening. Verify with checkOrdering below before trusting it.
# Three gates, all as a fraction of the collinear triple's image length.
# Validated by Monte-Carlo over random poses (see README). With these values
# mislabelling is 0.00% provided the camera sits at least 2x the wand length
# away; about 15% of frames are rejected, which is the honest cost of not
# guessing. A rejected frame sends n_valid=0 and becomes NaN in the CSV.
COLLINEAR_MAX_RESID = 0.15   # B must lie ON the line
D_OFFLINE_MIN = 0.10         # D must lie clearly OFF it
AC_RATIO_MIN = 1.2           # |AB|/|BC| must be decisive (design value 2.0)

STATUS_HZ = 2.0
GC_EVERY = 20

# ---------------------------------------------------------------------------
cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(FRAMESIZE)
cam.snapshot(time=1000)
cam.auto_gain(False, gain_db=GAIN_DB)
cam.auto_exposure(False, exposure_us=EXPOSURE_US)
cam.snapshot(time=3000)

_x0, _y0, sensor_w, sensor_h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (0, 0, sensor_w, sensor_h))

thresh = [(THRESH_LO, 255)]
_geom = None
_ratio = 1.0


def refresh_geom():
    # sensor_x = (image_x - W/2) * ratio + offx    (same as kalmanFilter.py)
    global _geom, _ratio
    x, y, w, h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
    W = cam.width()
    H = cam.height()
    ratio = min(w / float(W), h / float(H))
    offx = (w - (W * ratio)) / 2.0 + x + (sensor_w / 2.0)
    offy = (h - (H * ratio)) / 2.0 + y + (sensor_h / 2.0)
    _geom = (W, H, w, h, ratio, offx, offy)
    _ratio = ratio
    return _geom


def image_to_sensor(cx, cy):
    W, H, w, h, ratio, offx, offy = _geom
    return ((cx - W * 0.5) * ratio + offx,
            (cy - H * 0.5) * ratio + offy)


refresh_geom()
print("sensor %dx%d | image %dx%d | ratio %.3f"
      % (sensor_w, sensor_h, _geom[0], _geom[1], _ratio))


def get_buffer(img):
    try:
        return img.bytearray()
    except (AttributeError, MemoryError):
        return None


def refine_centroid(buf, stride, W, H, bx, by, bw, bh):
    # Intensity-weighted centroid over the blob box + REFINE_PAD.
    x0 = bx - REFINE_PAD
    y0 = by - REFINE_PAD
    x1 = bx + bw + REFINE_PAD
    y1 = by + bh + REFINE_PAD
    clipped = 0
    if x0 < 0:
        x0 = 0
        clipped = 1
    if y0 < 0:
        y0 = 0
        clipped = 1
    if x1 > W:
        x1 = W
        clipped = 1
    if y1 > H:
        y1 = H
        clipped = 1
    if (x1 - x0) * (y1 - y0) > REFINE_MAX_PX:
        return None
    sw = 0.0
    sx = 0.0
    sy = 0.0
    npix = 0
    nsat = 0
    floor = REFINE_FLOOR
    sat = SAT_LEVEL
    y = y0
    while y < y1:
        base = y * stride
        rw = 0
        rx = 0
        x = x0
        while x < x1:
            p = buf[base + x]
            if p > floor:
                wgt = p - floor
                rw += wgt
                rx += wgt * x
                npix += 1
                if p >= sat:
                    nsat += 1
            x += 1
        if rw:
            sw += rw
            sx += rx
            sy += rw * y
        y += 1
    if sw <= 0.0 or npix < 3:
        return None
    # +0.5 matches find_blobs' pixel-centre convention
    return (sx / sw + 0.5, sy / sw + 0.5, npix, nsat, clipped)


def centroid_sigma(npix, nsat, ratio):
    # 1-sigma in SENSOR px. sqrt of kalmanFilter.py detection_R().
    if npix < 1:
        npix = 1
    var = (CENTROID_SIGMA * CENTROID_SIGMA) * (AREA_NOMINAL_SENSOR / npix)
    if nsat > 0.25 * npix:
        var *= SAT_R_INFLATE
    s = math.sqrt(var) * ratio
    if s < SIGMA_MIN:
        s = SIGMA_MIN
    elif s > SIGMA_MAX:
        s = SIGMA_MAX
    return s


# --- ordering --------------------------------------------------------------
def _d2(p, q):
    dx = p[0] - q[0]
    dy = p[1] - q[1]
    return dx * dx + dy * dy


def _line_dist(p, a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    L = math.sqrt(dx * dx + dy * dy)
    if L == 0.0:
        return 0.0
    return abs(dy * (p[0] - a[0]) - dx * (p[1] - a[1])) / L


def order_markers(pts):
    # pts: exactly 4 (u, v) tuples in arbitrary order.
    # Returns [iA, iB, iC, iD] as indices into pts, or None if the frame does
    # not look like the rig (bad detection, occlusion, spurious blob).
    if len(pts) != 4:
        return None
    best = None
    for i in range(4):                                  # try each point as D
        rest = [j for j in range(4) if j != i]
        pairs = ((rest[0], rest[1]), (rest[0], rest[2]), (rest[1], rest[2]))
        e1, e2 = pairs[0]
        span = _d2(pts[e1], pts[e2])
        for p in pairs[1:]:                             # ends = furthest pair
            s = _d2(pts[p[0]], pts[p[1]])
            if s > span:
                span = s
                e1, e2 = p
        mid = rest[0]
        for j in rest:
            if j != e1 and j != e2:
                mid = j
        resid = _line_dist(pts[mid], pts[e1], pts[e2])
        if best is None or resid < best[0]:
            best = (resid, e1, e2, mid, i, math.sqrt(span))
    resid, e1, e2, mid, d, L = best
    if L <= 0.0:
        return None
    if (resid / L) > COLLINEAR_MAX_RESID:
        return None                          # B not on the line -> not the rig
    if (_line_dist(pts[d], pts[e1], pts[e2]) / L) < D_OFFLINE_MIN:
        return None                          # D projected onto the line: the
                                             # D/B roles are not separable here
    dA = _d2(pts[e1], pts[mid])
    dC = _d2(pts[e2], pts[mid])
    lo, hi = (dC, dA) if dA > dC else (dA, dC)
    if lo <= 0.0 or hi < (AC_RATIO_MIN * AC_RATIO_MIN) * lo:
        return None                          # A/C call too close to make
    if dA < dC:
        e1, e2 = e2, e1                      # e1 = end farther from B = A
    return [e1, mid, e2, d]


# --- detection -------------------------------------------------------------
def scan_frame(img, buf):
    # Full field scan. Returns [(u, v, sigma, flags), ...] in A,B,C,D order,
    # or None. Coordinates are SENSOR px.
    W, H, w, h, ratio, offx, offy = _geom
    at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (ratio * ratio)))
    blobs = img.find_blobs(thresh, area_threshold=at,
                           pixels_threshold=at, merge=False)
    if len(blobs) < N_MARKERS:
        return None
    blobs = sorted(blobs, key=lambda b: b.pixels, reverse=True)[:N_MARKERS]
    raw = []
    for b in blobs:
        cx, cy = b.cxf, b.cyf
        flags = 0x08                                    # bit3 VALID
        npix = b.pixels
        nsat = 0
        if buf is not None:
            r = refine_centroid(buf, W, W, H, b.x, b.y, b.w, b.h)
            if r is not None:
                cx, cy, npix, nsat, clip = r
                flags |= 0x01                           # bit0 refined
                if nsat > 0:
                    flags |= 0x02                       # bit1 saturated
                if clip:
                    flags |= 0x04                       # bit2 clipped
        mx, my = image_to_sensor(cx, cy)
        raw.append((mx, my, centroid_sigma(npix, nsat, ratio), flags))
    idx = order_markers([(r[0], r[1]) for r in raw])
    if idx is None:
        return None
    return [raw[i] for i in idx]


# --- packet ----------------------------------------------------------------
# Little-endian, packed. 10-byte header + 13 bytes per marker.
#   B  magic = 0x4C            ('L' - distinct from the tracker's 0x4B)
#   B  cam_id
#   B  n_valid                 0 or N_MARKERS. 0 = frame missed the strobe.
#   B  n_markers
#   H  seq                     frame counter, wraps at 65535
#   I  t_us                    approx exposure midpoint, camera-local clock
#   per marker, in A,B,C,D order:
#     f u, f v                 refined centroid, SENSOR px
#     f sigma                  1-sigma, SENSOR px
#     B flags                  bit0 refined, bit1 sat, bit2 clipped, bit3 valid
#
# t_us is camera-local and unsynchronised. MATLAB groups packets into strobe
# pulses by host arrival time, which works because the frame period is well
# below the pulse period. t_us is carried for diagnostics only.
PKT_MAGIC = 0x4C
PACKET_FMT = '<BBBBHI' + 'fffB' * N_MARKERS
PACKET_SIZE = struct.calcsize(PACKET_FMT)
_pkt = bytearray(PACKET_SIZE)
_vals = [PKT_MAGIC, CAM_ID, 0, N_MARKERS, 0, 0] + [0.0] * (4 * N_MARKERS)

sock = None
target = None


def net_init():
    global sock, target
    import network
    import socket
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    print("Connecting to WiFi '%s'..." % WIFI_SSID)
    wlan.connect(WIFI_SSID, WIFI_KEY)
    t0 = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), t0) > 30000:
            raise OSError("WiFi connection timed out after 30 s")
        time.sleep_ms(100)
    print("Connected! IP: %s" % wlan.ifconfig()[0])
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    target = (HOST_IP, HOST_PORT)
    print("cam %d streaming %d-byte packets to %s:%d"
          % (CAM_ID, PACKET_SIZE, HOST_IP, HOST_PORT))
    return wlan


def send(seq, t_us, dets):
    # dets is None when the frame missed the strobe. Send anyway, n_valid=0,
    # so silence on the host means a dead camera rather than a dark frame.
    if sock is None:
        return
    _vals[2] = 0 if dets is None else N_MARKERS
    _vals[4] = seq & 0xFFFF
    _vals[5] = t_us & 0xFFFFFFFF
    k = 6
    if dets is None:
        for _ in range(N_MARKERS):
            _vals[k] = 0.0
            _vals[k + 1] = 0.0
            _vals[k + 2] = 0.0
            _vals[k + 3] = 0
            k += 4
    else:
        for d in dets:
            _vals[k] = d[0]
            _vals[k + 1] = d[1]
            _vals[k + 2] = d[2]
            _vals[k + 3] = d[3]
            k += 4
    struct.pack_into(PACKET_FMT, _pkt, 0, *_vals)
    try:
        sock.sendto(_pkt, target)
    except OSError:
        pass


# --- modes -----------------------------------------------------------------
def mode_expose():
    # Exposure is FIXED long. Tune the LED trimpot / drive current, not this.
    # Target: peak 200-230, sat 0, blobs = N_MARKERS, each blob w/h near 1.00.
    print("EXPOSE  exposure fixed at %d us - adjust the LED, not the camera."
          % EXPOSURE_US)
    while True:
        img = cam.snapshot()
        buf = get_buffer(img)
        refresh_geom()
        W, H = _geom[0], _geom[1]
        peak = 0
        nsat = 0
        if buf is not None:
            n = W * H
            i = 0
            while i < n:
                p = buf[i]
                if p > peak:
                    peak = p
                if p >= SAT_LEVEL:
                    nsat += 1
                i += 7                      # sparse sweep, plenty for a peak
        at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (_ratio * _ratio)))
        blobs = img.find_blobs(thresh, area_threshold=at,
                               pixels_threshold=at, merge=False)
        wh = ""
        if blobs:
            b = max(blobs, key=lambda z: z.pixels)
            wh = "  biggest %dx%d px=%d w/h=%.2f" % (
                b.w, b.h, b.pixels, b.w / float(b.h) if b.h else 0.0)
        print("peak=%3d sat=%4d blobs=%d%s" % (peak, nsat, len(blobs), wh))
        time.sleep_ms(300)


def mode_readout():
    # Measure t_readout. Hold everything still and strobe.
    # A short exposure lights only the rows integrating when the pulse lands:
    #     t_readout = EXPOSURE_US / band_fraction
    # Raise EXPOSURE_US until band_fraction reaches 1.00 - that exposure IS
    # t_readout, and is the cross-check on the division above.
    print("READOUT  exposure %d us. Strobe the markers, watch band_frac."
          % EXPOSURE_US)
    print("  band_frac < 1.00 -> t_readout = %d / band_frac us" % EXPOSURE_US)
    print("  band_frac = 1.00 -> exposure already exceeds t_readout")
    while True:
        img = cam.snapshot()
        buf = get_buffer(img)
        refresh_geom()
        W, H = _geom[0], _geom[1]
        if buf is None:
            print("no frame buffer")
            time.sleep_ms(300)
            continue
        first = -1
        last = -1
        y = 0
        while y < H:
            base = y * W
            x = 0
            hit = 0
            while x < W:
                if buf[base + x] > THRESH_LO:
                    hit = 1
                    break
                x += 4                       # stride 4 - blobs are wider
            if hit:
                if first < 0:
                    first = y
                last = y
            y += 1
        if first < 0:
            print("no lit rows - is the strobe running?")
        else:
            frac = (last - first + 1) / float(H)
            est = EXPOSURE_US / frac if frac > 0 else 0
            print("rows %d..%d of %d  band_frac=%.3f  -> t_readout ~ %d us"
                  % (first, last, H, frac, est))
        time.sleep_ms(300)


def mode_run():
    if ENABLE_UDP:
        try:
            net_init()
        except Exception as e:
            print("UDP disabled: %s" % e)
    seq = 0
    nhit = 0
    nrej = 0
    status_ms = int(1000.0 / STATUS_HZ) if STATUS_HZ > 0 else 0
    last_status = time.ticks_ms()
    print("RUN  cam %d, %d markers, exposure %d us" %
          (CAM_ID, N_MARKERS, EXPOSURE_US))
    while True:
        img = cam.snapshot()
        t_meas = (time.ticks_us() - EXPOSURE_US // 2) & 0xFFFFFFFF
        buf = get_buffer(img)
        refresh_geom()
        dets = scan_frame(img, buf)
        if dets is None:
            nrej += 1
        else:
            nhit += 1
        send(seq, t_meas, dets)
        seq += 1
        if GC_EVERY and (seq % GC_EVERY) == 0:
            gc.collect()
        if status_ms and time.ticks_diff(time.ticks_ms(), last_status) >= status_ms:
            last_status = time.ticks_ms()
            tot = nhit + nrej
            print("seq=%d  hits=%d (%.0f%%)  ratio=%.2f"
                  % (seq, nhit, 100.0 * nhit / tot if tot else 0.0, _ratio))


if MODE == "expose":
    mode_expose()
elif MODE == "readout":
    mode_readout()
elif MODE == "run":
    mode_run()
else:
    raise ValueError("unknown MODE: %s" % MODE)
