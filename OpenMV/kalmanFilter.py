# Kalman Filter - Shannon Pitman
# Multi-marker tracking on OpenMV v5 (csi API).
#
# Tuning:
#   1. MODE = "expose"  -> lower EXPOSURE_US until peak reads 200-230 with
#                          sat_px = 0, and check blobs=1 with w/h near 1.00 -
#                          the four LEDs in the diffuser must read as one round
#                          blob. Then walk the marker to MAX range and read
#                          SENSOR AREA -> set AREA_MIN_SENSOR to ~0.7x it.
#   2. MODE = "floor"   -> sweeps REFINE_FLOOR. Copy all THREE printed values
#                          (REFINE_FLOOR, CENTROID_SIGMA, AREA_NOMINAL_SENSOR)
#                          from the SAME row.
#   3. MODE = "measure" -> confirms sigma at the chosen floor. Optional.
#   4. MODE = "run" with DEBUG_TIMING = True -> watch NIS, target 2.0.
#                          NIS > 2 -> raise ACCEL_STD.  NIS < 2 -> lower it.
#
# LED / marker characterisation lives in ledCharacterise.py. It was MODE="led"
# here; it needs none of the filter, so it no longer ships with it.
#
#
# Filter in SENSOR coordinates only.
# Keep MAX_RATIO = 1.0 so the readout is never downscaled.
# Send RAW centroids only.

import csi
import time
import math
import gc
import struct

MODE = "expose"  # "expose" | "floor" | "measure" | "run" | "snaprate"

DEBUG_DRAW = False  # draw crosses on the frame. Costs fps.
DEBUG_TIMING = False  # per-stage microsecond report every TIMING_N frames
STATUS_HZ = 2.0  # console status rate in "run" mode (0 = silent)
TIMING_N = 100  # frames per timing report

ENABLE_UDP = False  # stream binary packets to MATLAB
CAM_ID = 2

WIFI_SSID = "msg.network"
WIFI_KEY = "msg.t66Yu9"
HOST_IP = "192.168.0.32"
HOST_PORT = 7007

# VARIABLE INPUTS: Characterised by expose and measure:
N_MARKERS = 1  # number of markers to track
# from MODE="expose"
EXPOSURE_US = 700     # set with MODE="expose". Shorter = less motion smear.
# from MODE="floor"
REFINE_FLOOR = 50   # background subtracted before weighting -> CALIBRATE with measure mode: lowest sigma
CENTROID_SIGMA = 0.0544  # IMAGE px -> tune at nominal pixel area
AREA_NOMINAL_SENSOR = 980  # Reference to scale R with blob size.
AREA_MIN_SENSOR = 120.0  # STILL NEED to calibrate: test area of marker at furthest distance


MEASURE_WIN_PX = 240  # 1:1 crop size used by MODE="measure"
GAIN_DB = 0

SEARCHING_RESOLUTION = csi.VGA
TRACKING_EDGE_TOLERANCE = 0.30  # recentre once the group drifts 30% off-centre

# Prevent the window from changing too regularly
WINDOW_MULTIPLE = 64  # multiple of readout w/h
WINDOW_SHRINK_SLACK = 0  # keep at 0. Larger values let ratio drift above 1 -> pixel size between sensor and image not constant
RES_HOLD_FRAMES = 30  # min frames between framesize changes
MAX_WINDOW_GATE = 150.0  # cap the gate's contribution to window padding
MAX_WINDOW_LEAD = 100.0  # cap the velocity-extrapolation padding
WINDOW_SETTLE = 1  # frames to flush after a window change.

RES_LADDER = [(csi.QQVGA, 160, 120),  # all 4:3
              (csi.QVGA,  320, 240),
              (csi.VGA,   640, 480)]

PIN_TRACKING_RES = None  # None = adaptive ladder. Pinning measured slower.
MAX_RATIO = 1.0  # 1.0 = readout never downscaled

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(SEARCHING_RESOLUTION)
cam.snapshot(time=1000)
cam.auto_gain(False, gain_db=GAIN_DB)
cam.auto_exposure(False, exposure_us=EXPOSURE_US)
cam.snapshot(time=3000)

# x, y = 0, w,h= physical sensor array
_x0, _y0, sensor_w, sensor_h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
print("sensor array %dx%d | full-readout ratio: VGA %.2f  QVGA %.2f  QQVGA %.2f"
      % (sensor_w, sensor_h, sensor_w / 640.0, sensor_w / 320.0, sensor_w / 160.0))

# Detection
THRESH_LO = 80
thresh = [(THRESH_LO, 255)]
MAX_DETS = 5  # preallocated detection slots per frame
ROW_TOL = 30  # sensor-px band for grouping markers into rows

MIN_AREA_IMG = 4

REFINE = True  # intensity-weighted centroid
REFINE_PAD = 2  # look 2 px around blob bounding box
REFINE_MAX_PX = 3000  # skip refinement above this box area (too slow)
SAT_LEVEL = 250  # saturated pixel value

# Filter tuning
# Initial covariance. P0_VEL generous
V_INIT_STD = 2000.0  # sensor px/s, 1-sigma speed a new track may have
P0_POS = 100.0  # px^2 -> know starting position to 10 pixels
P0_VEL = V_INIT_STD * V_INIT_STD  # don't know starting velocity

# Trust measured velocity rather
SEED_VELOCITY = True
SEED_MATCH_PX = 120.0  # max inter-frame motion accepted -> lower for multi-marker config
MISS_INFLATE = 1.6  # grow P each frame w/out measurement

# Process noise q = ACCEL_STD^2 * TAU_MANOEUVRE  (px^2/s^3).
ACCEL_STD = 800.0  # sensor px/s^2, RMS
TAU_MANOEUVRE = 0.05  # s, how long one acceleration holds
q = ACCEL_STD * ACCEL_STD * TAU_MANOEUVRE

# Measurement noise from MODE="measure"
R_MIN = 0.0009  # px^2 floor (0.03 px)
R_MAX = 100.0
R_NOM = CENTROID_SIGMA * CENTROID_SIGMA  # derived - never edit by hand
SAT_R_INFLATE = 6.0  # multiply R when the blob is saturated
REJECT_CLIPPED = True  # drop blobs touching the image edge

# Gating
CHI2_GATE = 9.21  # chi-square, 2 DOF, 99%
MAX_SEARCH_PX = 400.0  # cap to size the window
MAX_ROI_PX = 80.0  # cap on the per-track blob ROI -> scan pixels is timeous
gate_min = 10.0
gate_nSig = 5.0

ROImargin = 15  # px around the marker bounding box
MAX_MISSES = 10  # amount of misses before needing to reaquire
FALLBACK_MISSES = 2  # scanning full frame
DT_MAX = 0.25  # s
DT_NOMINAL = 0.02  # s, used only when dt comes back <= 0
V_MAX = 20000.0  # sensor px/s, divergence guard while coasting
DEDUP_PX = 3.0  # detections closer than this are the same blob

GC_EVERY = 20  # prevents automatic GC -> collect every N frames

current_res = SEARCHING_RESOLUTION

# Cached per frame: (W, H, w, h, ratio, offx, offy)
# sensor_x = (image_x - W/2) * ratio + offx
_geom = None
_ratio = 1.0


# Image to sensor convertions
def refresh_geom():
    # Call after ANY framesize or readout-window change, and once per frame.
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


def sensor_to_image(mx, my, g=None):
    W, H, w, h, ratio, offx, offy = g if g else _geom
    return (W * 0.5 + (mx - offx) / ratio,
            H * 0.5 + (my - offy) / ratio)


# CV Kalman filter, one per axis
class pixelAxis:
    __slots__ = ('p', 'v', 'P00', 'P01', 'P11')

    def __init__(self, p0):
        self.p, self.v = p0, 0.0
        self.P00, self.P01, self.P11 = P0_POS, 0.0, P0_VEL

    def predict(self, dt):
        self.p += dt * self.v  # constant velocity
        # covariance matrix from state transition
        P00n = self.P00 + 2.0 * dt * self.P01 + dt * dt * self.P11
        P01n = self.P01 + dt * self.P11
        P11n = self.P11
        # process noise
        P00n += q * dt * dt * dt / 3.0
        P01n += q * dt * dt / 2.0
        P11n += q * dt
        self.P00, self.P01, self.P11 = P00n, P01n, P11n
        return self.P00  # PRIOR position variance; S = P00 + R per detection

    def update(self, z, R):
        # measurement z (+ variance R) into the state.
        y = z - self.p  # innovation
        S = self.P00 + R  # innovation variance
        K0 = self.P00 / S  # gain: 0 = trust model, 1 = trust measurement
        K1 = self.P01 / S
        self.p += K0 * y
        self.v += K1 * y
        if self.v > V_MAX:  # divergence guard
            self.v = V_MAX
        elif self.v < -V_MAX:
            self.v = -V_MAX
        # Joseph form: P = (I-KH)P(I-KH)' + KRK'. Stays positive definite in
        # single-precision floats
        P00_old, P01_old, P11_old = self.P00, self.P01, self.P11
        A = 1.0 - K0
        self.P00 = A * A * P00_old + K0 * K0 * R
        self.P01 = A * (P01_old - K1 * P00_old) + K0 * K1 * R
        self.P11 = (P11_old - 2.0 * K1 * P01_old
                    + K1 * K1 * (P00_old + R))


class MarkerTrack:
    # One marker = two independent pixelAxis filters (u across, v down).
    # zu/zv/zR hold the RAW measurement for this frame -> send
    __slots__ = ('id', 'u', 'v', 'Pu', 'Pv', 'bw', 'bh', 'misses', 'flags',
                 'zu', 'zv', 'zR')

    def __init__(self, mid, u0, v0, bw=4, bh=4):
        self.id = mid
        self.u = pixelAxis(u0)
        self.v = pixelAxis(v0)
        self.Pu = P0_POS
        self.Pv = P0_POS
        self.bw = bw  # blob size in SENSOR px
        self.bh = bh
        self.misses = 0
        self.flags = 0
        self.zu = 0.0
        self.zv = 0.0
        self.zR = 0.0

    def predict(self, dt):
        self.Pu = self.u.predict(dt)
        self.Pv = self.v.predict(dt)
        if self.misses > 0 and MISS_INFLATE > 1.0:
            f = MISS_INFLATE
            # widen the search gate in case marker is just outside it
            self.u.P00 *= f
            self.u.P11 *= f
            self.v.P00 *= f
            self.v.P11 *= f
            self.Pu = self.u.P00
            self.Pv = self.v.P00

    def correct(self, u, v, bw, bh, R):
        self.zu = u  # keep unfiltered for the UDP packet
        self.zv = v
        self.zR = R
        self.u.update(u, R)
        self.v.update(v, R)
        self.bw = bw
        self.bh = bh
        self.misses = 0

    def gate(self):
        # Search radius in sensor px. Sizes ROIs and the readout window
        s = self.Pu + self.Pv + 2.0 * R_NOM
        if s < 0.0:
            s = 0.0
        g = gate_nSig * math.sqrt(s)
        if g < gate_min:
            g = gate_min
        elif g > MAX_SEARCH_PX:
            g = MAX_SEARCH_PX
        return g


# Readout window
def res_dims(res):
    # csi framesize constant -> (W, H), or None if not on the ladder
    for r, W, H in RES_LADDER:
        if r == res:
            return (W, H)
    return None


def pick_res(need_w, need_h, cur_res):
    # Smallest framesize that fits need_w x need_h at ratio <= MAX_RATIO
    if PIN_TRACKING_RES is not None:
        d = res_dims(PIN_TRACKING_RES)
        if d and need_w <= d[0] * MAX_RATIO and need_h <= d[1] * MAX_RATIO:
            return PIN_TRACKING_RES
    chosen = RES_LADDER[-1]
    for res, W, H in RES_LADDER:
        if need_w <= W * MAX_RATIO and need_h <= H * MAX_RATIO:
            chosen = (res, W, H)
            break
    res, W, H = chosen
    cur = res_dims(cur_res)
    if res != cur_res and cur is not None and W < cur[0]:
        # Shrinking: 20% margin
        if not (need_w <= W * MAX_RATIO * 0.8 and need_h <= H * MAX_RATIO * 0.8):
            return cur_res
    return res


_last_win = None  # (res, xc, yc, w, h) currently applied to the sensor
_res_hold = 0  # frames left before another framesize change is allowed

# Stall diagnostics. Reset every status print.
_n_reprog = 0   # IOCTL_SET_READOUT_WINDOW calls
_n_resize = 0   # cam.framesize() calls (the expensive one)


def reset_to_search():
    global current_res, _last_win
    if current_res != SEARCHING_RESOLUTION:
        cam.framesize(SEARCHING_RESOLUTION)
        current_res = SEARCHING_RESOLUTION
        cam.snapshot()  # discard the first frame after a resolution change
    cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (0, 0, sensor_w, sensor_h))  # Full sensor, search framesize
    _last_win = (SEARCHING_RESOLUTION, 0, 0, sensor_w, sensor_h)
    for _ in range(WINDOW_SETTLE):
        cam.snapshot()
    refresh_geom()


def fit_window(tracks, dt_ahead):
    # Centre + size (sensor px) of the predicted window holding all markers
    lead_u = tracks[0].u.v * dt_ahead
    lead_v = tracks[0].v.v * dt_ahead
    umin = umax = tracks[0].u.p + lead_u
    vmin = vmax = tracks[0].v.p + lead_v
    gate = gate_min
    half_blob = 0.0
    lead_sigma = 0.0
    # Bounding box
    for t in tracks:
        pu = t.u.p + t.u.v * dt_ahead
        pv = t.v.p + t.v.v * dt_ahead
        if pu < umin:
            umin = pu
        if pu > umax:
            umax = pu
        if pv < vmin:
            vmin = pv
        if pv > vmax:
            vmax = pv
        # How far the velocity error could carry the marker over dt_ahead -> pad window accordingly
        lv = dt_ahead * math.sqrt(t.u.P11 if t.u.P11 > t.v.P11 else t.v.P11)
        if lv > lead_sigma:
            lead_sigma = lv
        g = t.gate()
        if g > MAX_WINDOW_GATE:
            g = MAX_WINDOW_GATE
        if g > gate:
            gate = g
        hb = 0.5 * (t.bw if t.bw > t.bh else t.bh)
        if hb > half_blob:
            half_blob = hb
    if lead_sigma > MAX_WINDOW_LEAD:
        lead_sigma = MAX_WINDOW_LEAD
    pad = ROImargin + gate + half_blob + lead_sigma

    # Never let padding push the requirement past the largest framesize, or center_window() is forced into ratio > 1
    biggest = RES_LADDER[-1]
    mp_w = 0.5 * (biggest[1] * MAX_RATIO - (umax - umin))
    mp_h = 0.5 * (biggest[2] * MAX_RATIO - (vmax - vmin))
    mp = mp_w if mp_w < mp_h else mp_h
    if pad > mp:
        pad = mp
    if pad < ROImargin:
        pad = ROImargin
    return (0.5 * (umin + umax), 0.5 * (vmin + vmax),
            (umax - umin) + 2.0 * pad, (vmax - vmin) + 2.0 * pad,
            gate, half_blob * 2.0)


def _snap(v, lo, hi):
    # Round a window dimension UP to a multiple of 54 -> prevents reprogramming window every frame, clamped to [lo, hi].
    if v <= lo:
        return lo  # exactly the framesize: ratio stays 1.0
    v = ((int(v) + WINDOW_MULTIPLE - 1) // WINDOW_MULTIPLE) * WINDOW_MULTIPLE
    if v > hi:  # larger than the VGA
        v = hi
    return v


def center_window(cx, cy, need_w, need_h, res, verbose=False):
    global current_res, _last_win, _res_hold, _n_reprog, _n_resize
    dims = res_dims(res)
    W, H = dims if dims else (cam.width(), cam.height())
    if _res_hold > 0 and res != current_res:
        cur = res_dims(current_res)
        new = res_dims(res)
        if cur and new and new[0] < cur[0]:
            res = current_res
            W, H = cur
    s = max(1.0, need_w / float(W), need_h / float(H))
    w = _snap(min(int(W * s), sensor_w), W, sensor_w)
    h = _snap(min(int(H * s), sensor_h), H, sensor_h)

    # x,y are the window centre as an offset from the sensor centre
    x = int(cx - (sensor_w / 2.0))
    y = int(cy - (sensor_h / 2.0))
    xlim = (sensor_w - w) // 2
    ylim = (sensor_h - h) // 2
    xc = max(-xlim, min(xlim, x))
    yc = max(-ylim, min(ylim, y))

    if _last_win is not None:
        lres, lx, ly, lw, lh = _last_win
        big_enough = lw >= w and lh >= h
        not_wasteful = (lw - w) <= WINDOW_SHRINK_SLACK and (lh - h) <= WINDOW_SHRINK_SLACK
        centred = (abs(xc - lx) <= TRACKING_EDGE_TOLERANCE * lw and abs(yc - ly) <= TRACKING_EDGE_TOLERANCE * lh)
        if lres == res and big_enough and not_wasteful and centred:
            return False
        xlim = (sensor_w - w) // 2
        ylim = (sensor_h - h) // 2
        xc = max(-xlim, min(xlim, x))
        yc = max(-ylim, min(ylim, y))
    # If the window size needs to be altered
    try:
        if res != current_res:
            cam.framesize(res)
            current_res = res
            _res_hold = RES_HOLD_FRAMES
            _n_resize += 1
            cam.snapshot()  # discard the first frame after a resolution change

        cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (xc, yc, w, h))
        _n_reprog += 1
        _last_win = (res, xc, yc, w, h)

        for _ in range(WINDOW_SETTLE):
            cam.snapshot()
    except OSError as e:
        print("window %dx%d @ (%d,%d) rejected: %s -> falling back to search"
              % (w, h, xc, yc, e))
        reset_to_search()
        return True

    refresh_geom()

    if verbose and (xc != x or yc != y):
        print("edge clamp dx=%d dy=%d" % (x - xc, y - yc))
    return True  # window was reprogrammed


# Subpixel centroid: pixel can't be saturated -> mode expose to set correct exposure

def get_buffer(img):
    # Raw pixel bytes, or None if unavailable. Row y starts at y * stride.
    try:
        return img.bytearray()
    except (AttributeError, MemoryError):
        return None  # if none -> use blob centroid


def refine_centroid(buf, stride, W, H, bx, by, bw, bh):
    # Intensity-weighted centroid over the blob box + REFINE_PAD
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
        return None  # really big blobs -> use binary blob_centroid
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
        rw = 0  # row weight
        rx = 0  # row sum of wgt * x
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
    # +0.5 matches the pixel-centre convention find_blobs uses -> places at centre of a pixel not corner
    return (sx / sw + 0.5, sy / sw + 0.5, npix, nsat, clipped)


def detection_R(npix, nsat, ratio):
    # Per-detection measurement variance, sensor px^2
    # Centroid variance prop. to 1/N contributing pixels
    if npix < 1:
        npix = 1
    R = (CENTROID_SIGMA * CENTROID_SIGMA) * (AREA_NOMINAL_SENSOR / npix)
    if nsat > 0.25 * npix:
        R *= SAT_R_INFLATE  # saturated -> centroid less certain -> inflate R
    R *= ratio * ratio  # image px^2 -> sensor px^2
    if R < R_MIN:
        R = R_MIN
    elif R > R_MAX:
        R = R_MAX
    return R


# Detection gathering
#       [ u,   v  , bw,  bh, R, npix,flags]
_dets = [[0.0, 0.0, 0.0, 0.0, 1.0, 0, 0] for _ in range(MAX_DETS)]


def _add_det(n, b, buf, stride, W, H, ratio, offx, offy, halfW, halfH):
    # Fill slot n from blob b, converting to sensor px. Returns the new n.
    if n >= MAX_DETS:
        return n
    flags = 0
    cx = b.cxf
    cy = b.cyf
    npix = b.pixels
    nsat = 0
    if REFINE and buf is not None:
        r = refine_centroid(buf, stride, W, H, b.x, b.y, b.w, b.h)
        if r is not None:
            cx, cy, npix, nsat, clipped = r
            flags |= 1
            if nsat > 0.25 * npix:
                flags |= 2
            if clipped:
                flags |= 4
                if REJECT_CLIPPED:
                    return n  # edge blob: centroid is biased, drop it
    d = _dets[n]
    d[0] = (cx - halfW) * ratio + offx
    d[1] = (cy - halfH) * ratio + offy
    d[2] = b.w * ratio
    d[3] = b.h * ratio
    d[4] = detection_R(npix, nsat, ratio)
    d[5] = npix
    d[6] = flags
    return n + 1


def dedup(n):
    # Overlapping ROIs can report the same blob twice. Keep the one with more
    # contributing pixels (lower R). Returns the surviving count.
    i = 0
    while i < n:
        j = i + 1
        while j < n:
            du = _dets[i][0] - _dets[j][0]
            dv = _dets[i][1] - _dets[j][1]
            if du * du + dv * dv < DEDUP_PX * DEDUP_PX:
                drop = j if _dets[i][5] >= _dets[j][5] else i
                n -= 1
                if drop != n:
                    _dets[drop], _dets[n] = _dets[n], _dets[drop]
                if drop == i:
                    j = i + 1
                    continue
            else:
                j += 1
        i += 1
    return n


def find_detections(img, buf, tracks=None):
    # Fill _dets from this frame. Returns how many slots are valid.
    # Searches small ROIs around each track while tracking is healthy, and
    # falls back to a full-frame scan once any track has missed too often.
    W, H, w, h, ratio, offx, offy = _geom
    stride = W
    at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (ratio * ratio)))
    halfW = W * 0.5
    halfH = H * 0.5
    n = 0

    use_roi = tracks is not None and \
        all(t.misses <= FALLBACK_MISSES for t in tracks)

    if not use_roi:
        for b in img.find_blobs(thresh, area_threshold=at,
                                pixels_threshold=at, merge=False):
            n = _add_det(n, b, buf, stride, W, H, ratio, offx, offy,
                         halfW, halfH)
            if n >= MAX_DETS:
                break
        return dedup(n)

    for t in tracks:
        # Predicted position and search radius, converted to image px.
        ix = halfW + (t.u.p - offx) / ratio
        iy = halfH + (t.v.p - offy) / ratio
        g = t.gate()
        if g > MAX_ROI_PX:
            g = MAX_ROI_PX
        rad = (g + max(t.bw, t.bh)) / ratio
        # Clamp hard: an ROI even partly outside the image faults find_blobs.
        rx = int(ix - rad)
        ry = int(iy - rad)
        if rx < 0:
            rx = 0
        if ry < 0:
            ry = 0
        if rx >= W - 1 or ry >= H - 1:
            continue
        rw = int(2.0 * rad)
        rh = int(2.0 * rad)
        if rx + rw > W:
            rw = W - rx
        if ry + rh > H:
            rh = H - ry
        if rw < 2 or rh < 2:
            continue
        # Blob coords come back in absolute image coords, not ROI-relative.
        for b in img.find_blobs(thresh, roi=(rx, ry, rw, rh),
                                area_threshold=at, pixels_threshold=at,
                                merge=False):
            n = _add_det(n, b, buf, stride, W, H, ratio, offx, offy,
                         halfW, halfH)
            if n >= MAX_DETS:
                break
        if n >= MAX_DETS:
            break
    return dedup(n)


# ---------------------------------------------------------------------------
# Association - exact assignment over a Mahalanobis cost, small N
# ---------------------------------------------------------------------------
# Match tracks to detections as a SET, not one at a time. Greedy "nearest
# unclaimed blob" swaps IDs when two markers pass close, and once IDs swap
# everything downstream is wrong.
#
# Cost = squared Mahalanobis distance, so a coasting track (large P) accepts a
# wider spread. Leaving a track unassigned costs CHI2_GATE, so a match only
# wins if it sits inside the gate.

_cost = [0.0] * (8 * MAX_DETS)
_asg_cur = [-1] * 8
_asg_best = [-1] * 8
_asg_cost_best = [0.0]


def _dfs(ti, ntr, ndet, used, csum):
    # Depth-first search over assignments, pruned by the best cost so far.
    # `used` is a bitmask of claimed detections. Result lands in _asg_best.
    if csum >= _asg_cost_best[0]:
        return                       # prune: cannot beat the best
    if ti == ntr:
        _asg_cost_best[0] = csum     # all tracks placed - record it
        for k in range(ntr):
            _asg_best[k] = _asg_cur[k]
        return
    base = ti * MAX_DETS
    # option: leave this track unassigned
    _asg_cur[ti] = -1
    _dfs(ti + 1, ntr, ndet, used, csum + CHI2_GATE)
    di = 0
    while di < ndet:
        if not (used & (1 << di)):
            c = _cost[base + di]
            if c < CHI2_GATE:
                _asg_cur[ti] = di
                _dfs(ti + 1, ntr, ndet, used | (1 << di), csum + c)
        di += 1
    _asg_cur[ti] = -1


_nis_sum = 0.0
_nis_n = 0


def associate(tracks, ndet):
    # Match, then correct or coast each track. Also accumulates NIS.
    global _nis_sum, _nis_n
    ntr = len(tracks)
    for ti in range(ntr):
        t = tracks[ti]
        base = ti * MAX_DETS
        for di in range(ndet):
            d = _dets[di]
            R = d[4]
            du = d[0] - t.u.p
            dv = d[1] - t.v.p
            Su = t.Pu + R
            Sv = t.Pv + R
            _cost[base + di] = du * du / Su + dv * dv / Sv

    _asg_cost_best[0] = 1e30
    for k in range(ntr):
        _asg_best[k] = -1
    _dfs(0, ntr, ndet, 0, 0.0)

    for ti in range(ntr):
        t = tracks[ti]
        di = _asg_best[ti]
        if di >= 0:
            _nis_sum += _cost[ti * MAX_DETS + di]
            _nis_n += 1
            d = _dets[di]
            t.correct(d[0], d[1], d[2], d[3], d[4])
            t.flags = d[6] | 8          # bit3 = updated this frame
        else:
            t.misses += 1
            t.flags = 0                 # coasting - nothing to send to the base
            t.zu = 0.0
            t.zv = 0.0
            t.zR = 0.0


def scan_full(img, buf):
    # One full-frame scan for acquisition. Returns [(u, v, bw, bh), ...] in
    # SENSOR px - the biggest N_MARKERS blobs, ordered top->bottom then
    # left->right - or None if there are too few blobs.
    W, H, w, h, ratio, offx, offy = _geom
    at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (ratio * ratio)))
    blobs = img.find_blobs(thresh, area_threshold=at,
                           pixels_threshold=at, merge=False)
    if len(blobs) < N_MARKERS:
        return None
    blobs = sorted(blobs, key=lambda b: b.pixels, reverse=True)[:N_MARKERS]
    blobs.sort(key=lambda b: (int(b.cyf // ROW_TOL), b.cxf))
    out = []
    for b in blobs:
        cx, cy = b.cxf, b.cyf
        if REFINE and buf is not None:
            r = refine_centroid(buf, W, W, H, b.x, b.y, b.w, b.h)
            if r is not None:
                cx, cy = r[0], r[1]
        mx, my = image_to_sensor(cx, cy)
        out.append((mx, my, b.w * ratio, b.h * ratio))
    return out


# --- non-blocking acquisition ----------------------------------------------
# One scan per frame, driven by the main loop - never a blocking wait. The
# camera keeps running at frame rate and keeps sending packets (markers
# flagged invalid) the whole time it is searching, so the base can tell
# "alive, no lock" from "dead".

_acq_prev = None
_acq_prev_t = 0
_acq_frames = 0


def begin_acquire(reason):
    # Drop all tracks and go back to full-sensor search.
    global tracks, _acq_prev, _acq_frames
    print("ACQUIRE: %s" % reason)
    tracks = None
    _acq_prev = None
    _acq_frames = 0
    reset_to_search()


def try_acquire(img, buf, t_us):
    # One attempt. Returns a track list, or None if it needs more frames.
    # With SEED_VELOCITY it takes two frames: frame 1 is remembered, frame 2
    # gives the velocity by differencing.
    global _acq_prev, _acq_prev_t, _acq_frames, init_id
    _acq_frames += 1
    cur = scan_full(img, buf)
    if cur is None:
        _acq_prev = None
        return None

    if SEED_VELOCITY and _acq_prev is None:
        _acq_prev, _acq_prev_t = cur, t_us
        return None  # need a second frame to seed velocity

    dt_seed = time.ticks_diff(t_us, _acq_prev_t) / 1000000.0 if _acq_prev else 0.0
    ok_dt = 0.0 < dt_seed < DT_MAX
    new = []
    for i, d in enumerate(cur):
        t = MarkerTrack(i, d[0], d[1], d[2], d[3])
        if _acq_prev and ok_dt:
            bd2 = SEED_MATCH_PX * SEED_MATCH_PX
            bp = None
            for p in _acq_prev:  # nearest blob in the previous frame
                du = d[0] - p[0]
                dv = d[1] - p[1]
                dd = du * du + dv * dv
                if dd < bd2:
                    bd2 = dd
                    bp = p
            if bp is not None:
                t.u.v = (d[0] - bp[0]) / dt_seed
                t.v.v = (d[1] - bp[1]) / dt_seed
                # Two-point velocity variance = 2R/dt^2.
                pv = 2.0 * R_NOM / (dt_seed * dt_seed)
                if pv < P0_VEL:
                    t.u.P11 = pv
                    t.v.P11 = pv
        new.append(t)

    _acq_prev = None
    init_id = (init_id + 1) & 0xFF       # tells the base IDs were reassigned
    # WHAT THE BASE MUST DO WITH THIS, from a simulation result. The ids above
    # are BLOB ORDER and carry no claim - scan_full returns blobs in whatever
    # order it found them. That is correct and matches "send raw centroids
    # only", but it puts a hard requirement on the other end:
    #
    #   A base that re-solves correspondence by matching against its OWN
    #   predicted marker positions CANNOT absorb this when its prediction is
    #   wrong, and aggressive tracking then gets WORSE, not better.
    #
    # Measured in simulation on the most aggressive trajectory in the set:
    # giving the cameras this autonomous acquisition while the base resolved
    # identity by nearest-match against its own prediction took position RMS
    # from 30.7 mm to 7.1 m. Detection was never the problem - published
    # measurements per tick actually ROSE, from 1.14 to 3.34. Every full loss
    # renumbered the markers, and a momentarily imperfect prediction then
    # produced wrong labels which were fused under the wrong names. More
    # measurements, worse answer.
    #
    # So the base needs correspondence that does not depend on already knowing
    # the pose - triangulate the unlabelled blobs and match the constellation by
    # its pairwise-distance signature, as +initpose does. Until it has that,
    # expect a camera that re-acquires mid-flight to hurt rather than help, and
    # treat init_id changing as a reason to re-solve identity from scratch
    # rather than to carry the previous assignment forward.
    print("Acquired %d markers after %d frames (init_id=%d):"
          % (len(new), _acq_frames, init_id))
    for t in new:
        print("  id %d  u=%.2f v=%.2f  vu=%.0f vv=%.0f px/s"
              % (t.id, t.u.p, t.v.p, t.u.v, t.v.v))
    return new


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
# SEND RAW MEASUREMENTS ONLY - never t.u.p or t.u.P00. The base runs its own
# EKF, and feeding a filter with another filter's output double-counts
# information, hides the position/velocity correlation, and makes the base
# overconfident. While coasting there is no new information at all.
#
# Packet (little-endian, packed):
#   B  magic = 0x4B
#   B  cam_id
#   B  init_id    increments on every re-acquire. Marker IDs are only
#                 comparable across frames sharing an init_id.
#   B  n_markers
#   H  seq        frame counter, wraps at 65535
#   I  t_us       exposure-midpoint timestamp, us, wraps at ~71 min
#   then per marker:
#   f  u, f v     RAW refined centroid, SENSOR px
#   f  sigma      measurement 1-sigma, SENSOR px  (R = sigma^2)
#   B  flags      bit0 refined, bit1 saturated, bit2 clipped, bit3 VALID
#                 bit4 RESERVED - LABELS_VALID, see below. Do not reuse.
#
# Only bit3 decides whether to fuse. bit1/bit2 mean degraded but usable -
# sigma has already been inflated to match.
#
# BIT4 IS RESERVED ON A SIMULATION RESULT, and is deliberately NOT emitted,
# because this camera cannot honestly compute it yet. Written down so the bit is
# not spent on something else.
#
# init_id says "IDs were reassigned AT THIS MOMENT". What it cannot say is that
# identity has silently DRIFTED since - that the marker this camera still calls
# 3 is no longer the one the base calls 3, with no re-acquisition in between.
# The simulation added a per-frame test for exactly that: each live track is
# checked against the base's reprojected hint, and the camera declares its set
# unlabelled when they disagree. It caught cases a reassignment counter cannot.
#
# It needs the BASE->CAMERA hint, which this firmware does not have - grep for
# `hint` or `recv` and there is nothing. With no downlink there is no reference
# to drift against, and identity here is only ever reassigned wholesale in
# try_acquire, which init_id already covers exactly. So the correct firmware
# change today is to reserve the bit, not to fake it.
#
# WHEN THE DOWNLINK EXISTS, emit bit4 per marker: set when the track sits within
# the seeding radius of the hint for the marker it claims to be, clear
# otherwise. Per-frame and stateless - a latched flag goes stale once the base
# recovers, and stays unset if labels drift without a re-acquisition, which is
# the failure this is meant to catch.

PKT_MAGIC = 0x4B
PACKET_FMT = '<BBBBHI' + 'fffB' * N_MARKERS
PACKET_SIZE = struct.calcsize(PACKET_FMT)
_pkt = bytearray(PACKET_SIZE)
_vals = [PKT_MAGIC, CAM_ID, 0, N_MARKERS, 0, 0] + [0.0] * (4 * N_MARKERS)

sock = None
target = None
init_id = 0


def net_init():
    # Join WiFi and open a non-blocking UDP socket. Raises on timeout.
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
    sock.setblocking(False)   # never let a stalled link stall the tracker
    target = (HOST_IP, HOST_PORT)
    print("Streaming %d-byte packets to %s:%d" % (PACKET_SIZE, HOST_IP, HOST_PORT))
    return wlan


def send(seq, t_us, tracks):
    # Send one packet. tracks may be None while acquiring - send anyway, with
    # every marker flagged invalid, so silence still means "camera dead".
    if sock is None:
        return
    _vals[2] = init_id
    _vals[4] = seq & 0xFFFF
    _vals[5] = t_us & 0xFFFFFFFF
    if not tracks:
        k = 6
        for _ in range(N_MARKERS):
            _vals[k] = 0.0
            _vals[k + 1] = 0.0
            _vals[k + 2] = 0.0
            _vals[k + 3] = 0             # flags = 0 -> not a measurement
            k += 4
        struct.pack_into(PACKET_FMT, _pkt, 0, *_vals)
        try:
            sock.sendto(_pkt, target)
        except OSError:
            pass
        return
    k = 6
    for t in tracks:
        _vals[k] = t.zu                  # raw centroid, never t.u.p
        _vals[k + 1] = t.zv
        _vals[k + 2] = math.sqrt(t.zR) if t.zR > 0.0 else 0.0
        _vals[k + 3] = t.flags
        k += 4
    struct.pack_into(PACKET_FMT, _pkt, 0, *_vals)
    try:
        sock.sendto(_pkt, target)
    except OSError:
        pass  # buffer full / link down - dropping is correct, blocking is not


# ---------------------------------------------------------------------------
# Calibration modes
# ---------------------------------------------------------------------------

def mode_expose():
    # STEP 1. Lower EXPOSURE_US until peak reads 200-230 with 0 saturated px.
    # A clipped marker has a flat top and the weighted centroid loses its edge.
    reset_to_search()
    print("EXPOSURE CHECK - target peak 200-230, currently exposure_us=%d" % EXPOSURE_US)
    while True:
        img = cam.snapshot()
        buf = get_buffer(img)
        W, H = _geom[0], _geom[1]
        # Scale the area threshold by ratio^2 as everywhere else - at search
        # resolution ratio is ~4, so image area is ~16x smaller than sensor.
        _r = _geom[4]
        at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (_r * _r)))
        blobs = img.find_blobs(thresh, area_threshold=at,
                               pixels_threshold=at, merge=False)
        if not blobs:
            print("no blobs")
            time.sleep_ms(200)
            continue
        b = max(blobs, key=lambda z: z.pixels)
        peak = 0
        nsat = 0
        if buf is not None:
            y = max(0, b.y)
            while y < min(H, b.y + b.h):
                row = y * W
                x = max(0, b.x)
                while x < min(W, b.x + b.w):
                    p = buf[row + x]
                    if p > peak:
                        peak = p
                    if p >= SAT_LEVEL:
                        nsat += 1
                    x += 1
                y += 1
        # blobs/w/h are the blend check: a marker is four LEDs inside one
        # diffuser, and they must merge into ONE round blob. If they do not,
        # find_blobs returns several and every mode above takes the largest -
        # so sigma reads fine while the tracker follows a quarter of the marker.
        aspect = (b.w / float(b.h)) if b.h else 0.0
        print("peak=%3d  sat_px=%3d  img_px=%4d  SENSOR AREA=%5.0f  blobs=%d  w/h=%.2f  %s%s"
              % (peak, nsat, b.pixels, b.pixels * _r * _r, len(blobs), aspect,
                 "LOWER EXPOSURE" if nsat > 0 else
                 ("raise exposure" if peak < 180 else "OK"),
                 "  <- NOT BLENDING" if len(blobs) > 1 else
                 ("  <- LUMPY" if aspect < 0.9 or aspect > 1.1 else "")))
        time.sleep_ms(300)


def mode_measure(n=500):
    # STEP 2. Hold ONE marker rigidly still. Copy the two printed values into
    # AREA_NOMINAL_SENSOR and CENTROID_SIGMA.
    #
    # Two things this must get right:
    #  1. Measure at ratio 1, not at search resolution (ratio ~4) - so locate
    #     the marker first, then drop into a 1:1 crop.
    #  2. Accumulate deviations from a reference sample, not sum(x^2)/n - mean^2.
    #     Single-precision floats cancel catastrophically at u ~ 1600 and the
    #     naive form returns exactly 0.0000.
    reset_to_search()
    # Locate the LARGEST single blob. Deliberately does not use scan_full():
    # that requires N_MARKERS blobs, and this routine measures exactly one.
    loc = None
    seen = 0
    _r = _geom[4]
    at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (_r * _r)))
    for _ in range(200):
        img = cam.snapshot()
        buf = get_buffer(img)
        W, H = _geom[0], _geom[1]
        blobs = img.find_blobs(thresh, area_threshold=at,
                               pixels_threshold=at, merge=False)
        if not blobs:
            continue
        seen = len(blobs)
        b = max(blobs, key=lambda z: z.pixels)
        cx, cy = b.cxf, b.cyf
        if REFINE and buf is not None:
            r = refine_centroid(buf, W, W, H, b.x, b.y, b.w, b.h)
            if r is not None:
                cx, cy = r[0], r[1]
        loc = image_to_sensor(cx, cy)
        break
    if loc is None:
        print("R MEASUREMENT: no marker found at search resolution.")
        print("  ratio=%.2f  area threshold=%d image px  THRESH_LO=%d"
              % (_r, at, THRESH_LO))
        print("  blobs seen on the last frame: %d" % seen)
        print("  If the thresholder shows the marker, the blob is smaller than")
        print("  %d image px -> lower AREA_MIN_SENSOR (now %.0f) or raise exposure."
              % (at, AREA_MIN_SENSOR))
        return
    center_window(loc[0], loc[1], MEASURE_WIN_PX, MEASURE_WIN_PX, csi.QVGA)
    print("R MEASUREMENT - hold the marker still, %d samples..." % n)
    print("  measuring at ratio=%.3f, readout %dx%d (want ratio 1.000)"
          % (_geom[4], _geom[2], _geom[3]))
    su = sv = suu = svv = 0.0
    k = 0
    npix_sum = 0
    nb_max = 0          # worst blob count seen at ratio 1 - the real blend check
    u0 = None
    v0 = 0.0
    while k < n:
        img = cam.snapshot()
        buf = get_buffer(img)
        W, H = _geom[0], _geom[1]
        _r = _geom[4]
        at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (_r * _r)))
        blobs = img.find_blobs(thresh, area_threshold=at,
                               pixels_threshold=at, merge=False)
        if not blobs:
            continue
        if len(blobs) > nb_max:
            nb_max = len(blobs)
        b = max(blobs, key=lambda z: z.pixels)
        cx, cy, npix = b.cxf, b.cyf, b.pixels
        if REFINE and buf is not None:
            r = refine_centroid(buf, W, W, H, b.x, b.y, b.w, b.h)
            if r is not None:
                cx, cy, npix = r[0], r[1], r[2]
        if u0 is None:
            u0, v0 = cx, cy      # reference offset - keeps the sums O(1)
        du = cx - u0
        dv = cy - v0
        su += du
        sv += dv
        suu += du * du
        svv += dv * dv
        npix_sum += npix
        k += 1
    mu = su / k
    mv = sv / k
    varu = max(0.0, suu / k - mu * mu)
    varv = max(0.0, svv / k - mv * mv)
    npix_avg = npix_sum / float(k)
    sigma = math.sqrt(0.5 * (varu + varv))
    print("mean  u=%.3f v=%.3f  npix_avg=%.1f  ratio=%.3f  blobs_max=%d%s"
          % (u0 + mu, v0 + mv, npix_avg, _geom[4], nb_max,
             "  <- NOT BLENDING, the four LEDs are resolving separately"
             if nb_max > 1 else ""))
    print("sigma u=%.4f v=%.4f  sensor px" % (math.sqrt(varu), math.sqrt(varv)))
    print("-> set AREA_NOMINAL_SENSOR = %.0f" % npix_avg)
    print("-> set CENTROID_SIGMA      = %.4f" % sigma)
    print("   (CENTROID_SIGMA is quoted AT AREA_NOMINAL_SENSOR pixels, so if")
    print("    you set both from this run the rescale is the identity.)")
    print("NOTE: this includes marker/rig vibration. It is an upper bound.")


def mode_floor(n=150):
    # STEP 1b. Sweep REFINE_FLOOR and report centroid sigma at each. Lowest
    # RMS wins. Run AFTER MODE="expose" - the right floor sits just above the
    # background, and the background level depends on exposure.
    #
    # Hold ONE marker rigidly still for the whole sweep (~10 s).
    global REFINE_FLOOR
    saved = REFINE_FLOOR

    reset_to_search()
    loc = None
    _r = _geom[4]
    at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (_r * _r)))
    for _ in range(200):
        img = cam.snapshot()
        blobs = img.find_blobs(thresh, area_threshold=at,
                               pixels_threshold=at, merge=False)
        if not blobs:
            continue
        b = max(blobs, key=lambda z: z.pixels)
        loc = image_to_sensor(b.cxf, b.cyf)
        break
    if loc is None:
        print("FLOOR SWEEP: no marker found. Run MODE=\"expose\" first.")
        return

    center_window(loc[0], loc[1], MEASURE_WIN_PX, MEASURE_WIN_PX, csi.QVGA)
    W, H = _geom[0], _geom[1]
    at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (_geom[4] * _geom[4])))
    print("FLOOR SWEEP - hold the marker still. ratio=%.3f readout %dx%d"
          % (_geom[4], _geom[2], _geom[3]))

    # Background from the top rows - the marker is centred, so these are empty.
    img = cam.snapshot()
    buf = get_buffer(img)
    bs = bss = 0
    bn = 0
    if buf is not None:
        y = 0
        while y < 4:
            row = y * W
            x = 0
            while x < W:
                p = buf[row + x]
                bs += p
                bss += p * p
                bn += 1
                x += 1
            y += 1
    if bn:
        bmean = bs / float(bn)
        bsig = math.sqrt(max(0.0, bss / float(bn) - bmean * bmean))
        blimit = bmean + 3.0 * bsig
        print("background mean=%.1f sigma=%.1f -> floor must stay >= %.0f"
              % (bmean, bsig, blimit))
    else:
        blimit = 0.0
        print("background: buffer unavailable")

    print("")
    print("%6s %9s %9s %9s %8s" % ("FLOOR", "sig_u", "sig_v", "RMS", "npix"))
    best = None
    for f in (30, 40, 50, 60, 70, 80, 90, 100):
        REFINE_FLOOR = f
        su = sv = suu = svv = 0.0
        npix_sum = 0
        k = 0
        tries = 0
        u0 = None
        v0 = 0.0
        while k < n and tries < n * 3:
            tries += 1
            img = cam.snapshot()
            buf = get_buffer(img)
            if buf is None:
                continue
            blobs = img.find_blobs(thresh, area_threshold=at,
                                   pixels_threshold=at, merge=False)
            if not blobs:
                continue
            b = max(blobs, key=lambda z: z.pixels)
            r = refine_centroid(buf, W, W, H, b.x, b.y, b.w, b.h)
            if r is None:
                continue
            cx, cy, npix = r[0], r[1], r[2]
            if u0 is None:
                u0, v0 = cx, cy
            du = cx - u0
            dv = cy - v0
            su += du
            sv += dv
            suu += du * du
            svv += dv * dv
            npix_sum += npix
            k += 1
        if k < 10:
            print("%6d %9s" % (f, "no data"))
            continue
        mu = su / k
        mv = sv / k
        sgu = math.sqrt(max(0.0, suu / k - mu * mu))
        sgv = math.sqrt(max(0.0, svv / k - mv * mv))
        rms = math.sqrt(0.5 * (sgu * sgu + sgv * sgv))
        na = npix_sum / float(k)
        print("%6d %9.4f %9.4f %9.4f %8.0f" % (f, sgu, sgv, rms, na))
        if best is None or rms < best[1]:
            best = (f, rms, na)

    REFINE_FLOOR = saved
    print("")
    if best is None:
        print("no usable rows - check exposure and THRESH_LO")
        return
    print("-> REFINE_FLOOR        = %d" % best[0])
    print("-> CENTROID_SIGMA      = %.4f" % best[1])
    print("-> AREA_NOMINAL_SENSOR = %.0f" % best[2])
    print("   All three come from the SAME row. Never mix rows.")
    if best[0] < blimit:
        print("   WARNING: below background+3sigma (%.0f). Sigma can look good"
              % blimit)
        print("   while background noise drags the centroid toward the box centre.")
        print("   Prefer the lowest floor that is still >= %.0f." % blimit)


def mode_snaprate(n=300):
    # Sensor-limited or compute-limited? Times pure snapshot() with no
    # processing at each framesize. If these match your loop period, the
    # sensor is the bottleneck and optimising Python buys nothing.
    print("SENSOR THROUGHPUT - pure snapshot(), no processing")
    print("  exposure_us=%d  (exposure alone caps you at %.0f Hz)"
          % (EXPOSURE_US, 1e6 / max(EXPOSURE_US, 1)))
    for res, W, H in RES_LADDER:
        reset_to_search()
        cam.framesize(res)
        current = res
        cam.snapshot()
        cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (0, 0, W, H))
        for _ in range(5):
            cam.snapshot()          # let the pipeline settle
        t0 = time.ticks_us()
        for _ in range(n):
            cam.snapshot()
        el = time.ticks_diff(time.ticks_us(), t0)
        print("  %4dx%-4d readout 1:1 -> %6.0f us/frame  (%5.0f Hz)"
              % (W, H, el / float(n), 1e6 * n / max(el, 1)))
    print("If these match your loop period, the sensor is the bottleneck.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if MODE == "expose":
    mode_expose()
elif MODE == "measure":
    mode_measure()
    raise SystemExit
elif MODE == "floor":
    mode_floor()
    raise SystemExit
elif MODE == "snaprate":
    mode_snaprate()
    raise SystemExit

if N_MARKERS > 8:
    raise ValueError("_cost/_asg_cur/_asg_best are sized for <=8 tracks")

if ENABLE_UDP:
    try:
        wlan = net_init()
    except Exception as e:
        print("UDP disabled: %s" % e)
        wlan = None
else:
    wlan = None

clock = time.clock()
tracks = None
begin_acquire("startup")

# Prime the clock on a real frame so the first dt is meaningful.
cam.snapshot()
last_t = time.ticks_us()

seq = 0
_acc = [0, 0, 0, 0, 0, 0, 0]   # timing accumulators, us
_frames = 0
_tG_prev = time.ticks_us()     # end of previous loop, for the "other" bucket
_acc_dt = 0.0                  # summed true frame period over the report window
_det_errs = 0                  # frames where detection raised and was swallowed
_dt_clamps = 0
_last_status = time.ticks_ms()
_status_ms = int(1000.0 / STATUS_HZ) if STATUS_HZ > 0 else 0
nw = nh = 0
gate_dbg = blob_dbg = 0.0

# Per-status-window stall stats (cheap, always on).
SLOW_MS = 60          # a frame longer than this is a stall
_st_n = 0             # frames since last status print
_st_dt = 0.0          # summed true frame period, s
_st_dtmax = 0.0       # worst frame period, s
_st_snapmax = 0       # worst cam.snapshot(), us
_st_winmax = 0        # worst fit_window+center_window, us
_st_gcmax = 0         # worst gc.collect(), us
_st_slow = 0          # frames over SLOW_MS

while True:
    clock.tick()

    # -- snapshot FIRST, then timestamp ------------------------------------
    # Timestamping before snapshot propagates the state to a time before the
    # frame was exposed, lagging every estimate by half a frame period.
    tA = time.ticks_us()
    img = cam.snapshot()
    now = time.ticks_us()
    tB = time.ticks_us()

    dt = time.ticks_diff(now, last_t) / 1000000.0
    last_t = now
    if dt <= 0.0:
        dt = DT_NOMINAL
    elif dt > DT_MAX:
        # CLAMP, do not replace with a nominal dt. A GC pause really did eat
        # that time, so the gate must widen, not shrink.
        dt = DT_MAX
        _dt_clamps += 1

    # Exposure midpoint - the epoch this measurement belongs to.
    t_meas = time.ticks_add(now, -(EXPOSURE_US >> 1))

    # -- acquiring: one scan per frame, loop keeps running ------------------
    if tracks is None:
        buf = get_buffer(img)
        try:
            tracks = try_acquire(img, buf, now)
        except Exception as e:
            print("ACQUIRE ERROR %r" % (e,))
            tracks = None
        send(seq, t_meas, tracks)
        seq += 1
        if GC_EVERY and (seq % GC_EVERY) == 0:
            gc.collect()
        continue

    # -- 1. predict --------------------------------------------------------
    for t in tracks:
        t.predict(dt)
    tC = time.ticks_us()

    # -- 2. detect ---------------------------------------------------------
    buf = get_buffer(img)
    # Treat a bad frame as "no detections" - every track coasts one frame,
    # which the filter already handles - and print the exception so it is
    # diagnosable. Both field crashes landed in here.
    try:
        ndet = find_detections(img, buf, tracks)
    except Exception as e:
        _det_errs += 1
        print("DETECT ERROR %r | geom=%s buf=%s ndet->0"
              % (e, _geom, "None" if buf is None else len(buf)))
        ndet = 0
    tD = time.ticks_us()

    # -- 3. associate + correct --------------------------------------------
    associate(tracks, ndet)
    tE = time.ticks_us()

    # -- 4. send -----------------------------------------------------------
    send(seq, t_meas, tracks)
    seq += 1
    tF = time.ticks_us()

    if all(t.misses > MAX_MISSES for t in tracks):
        begin_acquire("all tracks lost")
        _acc = [0, 0, 0, 0, 0, 0, 0]
        _acc_dt = 0.0
        _frames = 0
        continue

    # -- 5. draw, then move the window -------------------------------------
    # Draw BEFORE center_window(): it calls refresh_geom(), and this image was
    # captured with the geometry current right now.
    if DEBUG_DRAW:
        for t in tracks:
            ix, iy = sensor_to_image(t.u.p, t.v.p)
            lx, ly = int(ix), int(iy)
            if 0 <= lx < _geom[0] and 0 <= ly < _geom[1]:
                img.draw_cross((lx, ly), color=255)
                img.draw_string((lx + 3, ly + 3), str(t.id), color=255)

    tF2 = time.ticks_us()
    cx, cy, nw, nh, gate_dbg, blob_dbg = fit_window(tracks,
                                                    dt * (1 + WINDOW_SETTLE))
    # Do NOT reset last_t if a settle frame is flushed here: the next frame
    # really is two frame periods away and dt must say so.
    center_window(cx, cy, nw, nh, pick_res(nw, nh, current_res))
    ww, wh = _geom[2], _geom[3]
    tG = time.ticks_us()

    if _res_hold > 0:
        _res_hold -= 1

    if GC_EVERY and (seq % GC_EVERY) == 0:
        _tgc = time.ticks_us()
        gc.collect()
        _gc_us = time.ticks_diff(time.ticks_us(), _tgc)
        if _gc_us > _st_gcmax:
            _st_gcmax = _gc_us

    # -- stall accounting (always on: it is a few us) ----------------------
    _st_n += 1
    _st_dt += dt
    if dt > _st_dtmax:
        _st_dtmax = dt
    if dt * 1000.0 > SLOW_MS:
        _st_slow += 1
    _snap_us = time.ticks_diff(tB, tA)
    if _snap_us > _st_snapmax:
        _st_snapmax = _snap_us
    _win_us = time.ticks_diff(tG, tF2)
    if _win_us > _st_winmax:
        _st_winmax = _win_us

    # -- reporting ---------------------------------------------------------
    if DEBUG_TIMING:
        _acc[0] += time.ticks_diff(tC, tB)  # kf predict
        _acc[1] += time.ticks_diff(tB, tA)  # snapshot
        _acc[2] += time.ticks_diff(tD, tC)  # find_blobs + centroid + mapping
        _acc[3] += time.ticks_diff(tE, tD)  # associate + kf update
        _acc[4] += time.ticks_diff(tG, tF)  # windowing + draw
        _acc[5] += time.ticks_diff(tF, tE)  # udp send
        # "other" = everything outside tA..tG: gc.collect(), this print,
        # clock.tick(). Without it the stages sum to 4 ms while the loop
        # actually runs at 15 ms and nothing says so.
        _acc[6] += time.ticks_diff(tA, _tG_prev)
        _tG_prev = tG
        # dt is the true frame period, snapshot-return to snapshot-return.
        # Trust it over clock.fps(), which disagreed by 30%.
        _acc_dt += dt
        _frames += 1
        if _frames >= TIMING_N:
            a = [v / _frames for v in _acc]
            kf = a[0] + a[3]
            loop = sum(a)
            nis = (_nis_sum / _nis_n) if _nis_n else -1.0
            dt_us = 1e6 * _acc_dt / _frames
            print("[us] kf=%.0f snap=%.0f blob=%.0f win=%.0f net=%.0f other=%.0f "
                  "| sum=%.0f  TRUE dt=%.0f us (%.0f Hz) clk=%.0f Hz "
                  "| NIS=%.2f (target 2.0) dtclamp=%d deterr=%d "
                  "| win=%dx%d ratio=%.2f need=%dx%d gate=%.0f blob=%.0f"
                  % (kf, a[1], a[2], a[4], a[5], a[6], loop,
                     dt_us, 1e6 / max(dt_us, 1), clock.fps(), nis, _dt_clamps, _det_errs,
                     ww, wh, _geom[4], int(nw), int(nh), gate_dbg, blob_dbg))
            _acc = [0, 0, 0, 0, 0, 0, 0]
            _acc_dt = 0.0
            _frames = 0
            _nis_sum = 0.0
            _nis_n = 0
            _dt_clamps = 0
    elif _status_ms:
        # Rate-limited. A per-frame print at 100+ Hz costs more than the filter.
        if time.ticks_diff(time.ticks_ms(), _last_status) >= _status_ms:
            _last_status = time.ticks_ms()
            nis = (_nis_sum / _nis_n) if _nis_n else -1.0
            # fps from the measured frame period, NOT clock.fps(): that one is
            # a cumulative average over only the frames it is called on, so it
            # is both stale and biased towards whatever the slow frames cost.
            fps = (_st_n / _st_dt) if _st_dt > 0.0 else 0.0
            print("fps=%.1f worst=%.0fms slow=%d/%d "
                  "snapmax=%.0fms winmax=%.0fms gcmax=%.0fms reprog=%d resize=%d "
                  "| win=%dx%d need=%dx%d gate=%.0f NIS=%.2f | "
                  % (fps, 1000.0 * _st_dtmax, _st_slow, _st_n,
                     _st_snapmax / 1000.0, _st_winmax / 1000.0,
                     _st_gcmax / 1000.0, _n_reprog, _n_resize,
                     ww, wh, int(nw), int(nh), gate_dbg, nis) +
                  " ".join("id%d(%.2f,%.2f)m%d" % (t.id, t.u.p, t.v.p, t.misses)
                           for t in tracks))
            _nis_sum = 0.0
            _nis_n = 0
            _st_n = 0
            _st_dt = 0.0
            _st_dtmax = 0.0
            _st_snapmax = 0
            _st_winmax = 0
            _st_gcmax = 0
            _st_slow = 0
            _n_reprog = 0
            _n_resize = 0
