# checkGeometry.py - Shannon Pitman
# Sanity check for the image <-> sensor mapping in kalmanFilter.py.
#
# WHAT IT PROVES
#   A stationary marker must report the SAME sensor coordinate no matter
#   which framesize or readout window it was seen through. If it does not,
#   the affine map in refresh_geom() is wrong and every downstream number
#   (intrinsics, pose, association) is wrong with it.
#
# HOW TO RUN
#   1. Clamp the camera. Clamp one bright marker in view. Nothing moves.
#   2. Match EXPOSURE_US / GAIN_DB / THRESH_LO / REFINE_FLOOR to kalmanFilter.py.
#   3. Run. Read the du/dv columns.
#
# HOW TO READ THE RESULT
#   SENSOR U/V = where THIS config thinks the marker is, in sensor px.
#             Every row is measuring one stationary marker, so this column
#             should read the same number all the way down. It is the whole
#             test in one place - du/dv are just it minus the reference.
#   du/dv  = this config's sensor coord minus the REFERENCE sensor coord.
#   dix/diy = the same error in IMAGE px. Where the marker actually landed
#             in this image, minus where the reference sensor coord says it
#             should land HERE. Sensor px is what feeds intrinsics and pose;
#             image px is what you size ROIs in.
#   START  = top-left SENSOR pixel actually being read out, derived from the
#             centre-relative window: (sensor_w/2 + x - w/2, ...). C4,R4 is
#             that start mod 4. Sensors align the readout start to a 2 or 4
#             px grid; a row whose C4,R4 differs from its neighbours' is the
#             first place to look when its du/dv differs from theirs.
#   sig u/v = frame-to-frame scatter of the SENSOR coord within this one
#             config, over SAMPLES frames, PER AXIS. Precision, not
#             accuracy - it says how reproducible the centroid is, never
#             whether the map is right. Its job is to be the yardstick: a
#             du of 2.0 px next to a sig of 0.03 is systematic, not noise.
#             The mean is known ~sqrt(SAMPLES) better again. Split by axis
#             because a marker that is smeared, clipped or lit unevenly is
#             noisier along one axis only, and so is a sensor that is
#             quantising one axis of the readout.
#   npix/sat = pixels above REFINE_FLOOR, and how many hit SAT_LEVEL, in the
#             worst frame of the run. sat must be 0. A blob that saturates
#             or changes size between configs moves its own centroid, which
#             looks exactly like a geometry error and is not one.
#
#   PASS  every 1:1 row agrees within ~1 sensor px. SPREAD is small.
#   FAIL  a row is out by tens or hundreds of px.
#         Check REQ vs GOT first - a driver silently changing the window
#         explains most failures and is not a bug in the maths.
#         BUT: IOCTL_GET_READOUT_WINDOW echoes back what it was given, so
#         GOT == REQ even when the sensor quietly moved the window. The
#         ALIGNMENT SWEEP at the end is what actually catches that.

import csi
import math

# --- match these to kalmanFilter.py ----------------------------------------
EXPOSURE_US = 1000  # keep equal to kalmanFilter.py - set there by MODE="expose"
GAIN_DB = 0
THRESH_LO = 80
thresh = [(THRESH_LO, 255)]
AREA_MIN_SENSOR = 120.0
MIN_AREA_IMG = 4
REFINE = True
REFINE_PAD = 2
REFINE_FLOOR = 50
REFINE_MAX_PX = 3000
SAT_LEVEL = 250
WINDOW_SETTLE = 1

RES_LADDER = [(csi.QQVGA, 160, 120),
              (csi.QVGA,  320, 240),
              (csi.VGA,   640, 480)]
RES_NAME = {csi.QQVGA: "QQVGA", csi.QVGA: "QVGA", csi.VGA: "VGA"}

# --- test settings ---------------------------------------------------------
SAMPLES = 20        # frames averaged per configuration
SETTLE_FRAMES = 3   # extra frames flushed after every reconfiguration
PASS_PX = 1.0       # sensor px. Spread above this is suspicious.
WIDTH = 127         # printed page width

# Alignment sweep. Walks the readout window one sensor px at a time and
# watches whether the marker's SENSOR coordinate holds still. See stage 4.
SWEEP = True
SWEEP_SPAN = 6      # steps each side of centre, 1 sensor px per step
SWEEP_SAMPLES = 8   # frames per step. Below SAMPLES - there are many steps.
SWEEP_RES = [csi.VGA, csi.QQVGA]   # largest and smallest 1:1 window

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(csi.VGA)
cam.snapshot(time=1000)
cam.auto_gain(False, gain_db=GAIN_DB)
cam.auto_exposure(False, exposure_us=EXPOSURE_US)
cam.snapshot(time=3000)

_x0, _y0, sensor_w, sensor_h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)

_geom = None
_ratio = 1.0

# ---------------------------------------------------------------------------
# COPIED VERBATIM from kalmanFilter.py - do not "improve" these here
# ---------------------------------------------------------------------------


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


def get_buffer(img):
    try:
        return img.bytearray()
    except (AttributeError, MemoryError):
        return None


def refine_centroid(buf, stride, W, H, bx, by, bw, bh):
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
    return (sx / sw + 0.5, sy / sw + 0.5, npix, nsat, clipped)


# ---------------------------------------------------------------------------
# Test rig
# ---------------------------------------------------------------------------


def find_marker(img, buf):
    # Biggest blob in the current frame. Returns
    # (img_x, img_y, npix, nsat, clipped) in IMAGE px, or None.
    # nsat = -1 means the refine path did not run, so saturation is unknown
    # and the blob's own binary centroid was used instead.
    W, H, w, h, ratio, offx, offy = _geom
    at = max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (ratio * ratio)))
    blobs = img.find_blobs(thresh, area_threshold=at,
                           pixels_threshold=at, merge=False)
    if not blobs:
        return None
    b = max(blobs, key=lambda z: z.pixels)
    cx, cy, npix, nsat, clipped = b.cxf, b.cyf, b.pixels, -1, 0
    if REFINE and buf is not None:
        r = refine_centroid(buf, W, W, H, b.x, b.y, b.w, b.h)
        if r is not None:
            cx, cy, npix, nsat, clipped = r[0], r[1], r[2], r[3], r[4]
    return (cx, cy, npix, nsat, clipped)


def win_start(x, y, w, h):
    # Top-left SENSOR pixel of a centre-relative readout window. This, not
    # (x, y), is what the sensor has to align to its readout grid.
    return (sensor_w // 2 + x - w // 2, sensor_h // 2 + y - h // 2)


def start_str(win):
    sx, sy = win_start(win[0], win[1], win[2], win[3])
    return ("(%d,%d)" % (sx, sy), "%d,%d" % (sx % 4, sy % 4))


def measure(n):
    # Average over n frames. Deviations from a reference sample keep every
    # term O(1) - the naive sum(x^2)/n - mean^2 form cancels catastrophically
    # in single-precision floats at sensor coords of ~1600.
    # Returns (iu, iv, mu, mv, sig_u, sig_v, npix, nsat, hits, clipped)
    # or None.
    # npix is the mean over the run, nsat the WORST frame - one saturated
    # frame is enough to bias the intensity-weighted centroid.
    su = sv = suu = svv = 0.0
    siu = siv = 0.0
    u0 = v0 = None
    npix_sum = 0
    nsat_max = -1
    clip = 0
    k = 0
    tries = 0
    while k < n and tries < n * 4:
        tries += 1
        img = cam.snapshot()
        buf = get_buffer(img)
        f = find_marker(img, buf)
        if f is None:
            continue
        ix, iy, npix, nsat, clipped = f
        mx, my = image_to_sensor(ix, iy)
        if u0 is None:
            u0, v0 = mx, my
        du = mx - u0
        dv = my - v0
        su += du
        sv += dv
        suu += du * du
        svv += dv * dv
        siu += ix
        siv += iy
        npix_sum += npix
        if nsat >= 0 and nsat > nsat_max:
            nsat_max = nsat
        clip += clipped
        k += 1
    if k < 3:
        return None
    mu = su / k
    mv = sv / k
    varu = max(0.0, suu / k - mu * mu)
    varv = max(0.0, svv / k - mv * mv)
    return (siu / k, siv / k, u0 + mu, v0 + mv,
            math.sqrt(varu), math.sqrt(varv),
            npix_sum / float(k), nsat_max, k, clip)


def apply_config(res, xc, yc, w, h):
    # Set framesize + readout window, flush, refresh geometry.
    # Returns (ok, requested, actual). The sensor may silently change the
    # window it was given - that difference is the first thing to check
    # when a row fails.
    req = (xc, yc, w, h)
    try:
        cam.framesize(res)
        cam.snapshot()
        cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, req)
        for _ in range(WINDOW_SETTLE + SETTLE_FRAMES):
            cam.snapshot()
    except OSError as e:
        print("    window %s REJECTED: %s" % (str(req), e))
        return (False, req, None)
    refresh_geom()
    got = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
    return (True, req, got)


def clamp_window(mx, my, w, h, frac_x=0.0, frac_y=0.0):
    # Window centre offset from sensor centre, placing the marker at
    # (0.5 - frac) of the way across the window. frac 0 = dead centre.
    cxa = mx + frac_x * w
    cya = my + frac_y * h
    x = int(cxa - sensor_w / 2.0)
    y = int(cya - sensor_h / 2.0)
    xlim = (sensor_w - w) // 2
    ylim = (sensor_h - h) // 2
    if xlim < 0 or ylim < 0:
        return None
    return (max(-xlim, min(xlim, x)), max(-ylim, min(ylim, y)), w, h)


HDR_FMT = ("%-19s %6s %-12s %-11s %5s %8s %8s %7s %7s %6s %6s "
           "%5s %5s %5s %3s")
ROW_FMT = ("%-19s %6.3f %-12s %-11s %5s %8.2f %8.2f %+7.2f %+7.2f "
           "%+6.2f %+6.2f %5.3f %5.3f %5.0f %3s")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

print("=" * WIDTH)
print("GEOMETRY CHECK")
print("=" * WIDTH)
print("SENSOR ARRAY   %d x %d  (%.2f MP)"
      % (sensor_w, sensor_h, sensor_w * sensor_h / 1e6))
print("exposure=%d us  gain=%d dB  thresh=%d  refine_floor=%d  samples=%d"
      % (EXPOSURE_US, GAIN_DB, THRESH_LO, REFINE_FLOOR, SAMPLES))
print("")
print("What each configuration costs you, before any measurement:")
print("  %-24s %-14s %7s  %10s  %s"
      % ("CONFIG", "READOUT", "RATIO", "FOV", "1 img px ="))
for _res, _W, _H in RES_LADDER[::-1]:
    _nm = RES_NAME[_res]
    _r = min(sensor_w / float(_W), sensor_h / float(_H))
    print("  %-24s %-14s %7.3f  %9.1f%%  %.2f sensor px"
          % ("%s, full sensor" % _nm, "%dx%d" % (sensor_w, sensor_h), _r,
             100.0, _r))
for _res, _W, _H in RES_LADDER[::-1]:
    _nm = RES_NAME[_res]
    _fov = 100.0 * (_W * _H) / float(sensor_w * sensor_h)
    print("  %-24s %-14s %7.3f  %9.1f%%  %.2f sensor px"
          % ("%s, 1:1 window" % _nm, "%dx%d" % (_W, _H), 1.0, _fov, 1.0))
print("")
print("Ratio is readout/framesize. Full readout = LARGEST ratio (no crop, all")
print("downscale). Windowing down to the framesize is what makes it 1.000.")
print("=" * WIDTH)

# --- stage 1: bootstrap. Find the marker anywhere on the array. -------------
# Full sensor is the only configuration guaranteed to contain the marker, but
# ratio is ~4 here so this position is COARSE. It is used only to aim stage 2.
ok, req, got = apply_config(csi.VGA, 0, 0, sensor_w, sensor_h)
if not ok:
    raise SystemExit("cannot set full-sensor readout - cannot locate the marker")
boot = measure(SAMPLES // 2)
if boot is None:
    raise SystemExit("no marker found at full sensor. Check exposure/threshold/aim.")
print("locate     full sensor @ VGA   ratio=%.3f -> approx SENSOR (%.1f, %.1f)"
      % (_geom[4], boot[2], boot[3]))

# --- stage 2: reference. Re-measure at ratio 1.000, largest window that fits.
# A reference measured at ratio 4 is ~4x coarser in sensor px, which would put
# noise into every du/dv. Measure the anchor where the pixels are 1:1.
ref = None
ref_name = None
ref_win = None
for res, W, H in RES_LADDER[::-1]:          # VGA first, largest 1:1 window
    win = clamp_window(boot[2], boot[3], W, H, 0.0, 0.0)
    if win is None:
        continue
    ok, req, got = apply_config(res, win[0], win[1], win[2], win[3])
    if not ok:
        continue
    if abs(_geom[4] - 1.0) > 0.01:          # insist on true 1:1
        continue
    m = measure(SAMPLES)
    if m is None:
        continue
    ref, ref_name, ref_win = m, "%s 1:1 centred" % RES_NAME[res], got
    break

if ref is None:
    raise SystemExit("could not establish a ratio-1.000 reference. Is the marker "
                     "too close to the sensor edge?")

REF_U, REF_V = ref[2], ref[3]
_rs, _ra = start_str(ref_win)
print("REFERENCE  %-18s ratio=%.3f  window %s  start %s  C4,R4 %s"
      % (ref_name, _geom[4], str(ref_win), _rs, _ra))
print("           image (%.2f, %.2f)  ->  SENSOR (%.2f, %.2f)"
      % (ref[0], ref[1], REF_U, REF_V))
print("           sig u=%.3f  sig v=%.3f  npix=%.0f  sat=%d"
      % (ref[4], ref[5], ref[6], ref[7]))
print("           Aimed with the COARSE full-sensor fix, so this window need")
print("           not equal the '%s' row below. Compare the two" % ref_name)
print("           START values before reading anything into that row's du/dv.")
print("-" * WIDTH)
print(HDR_FMT % ("CONFIG", "RATIO", "WIN x,y", "START", "C4,R4", "SENSOR U",
                 "SENSOR V", "du", "dv", "dix", "diy", "sig u", "sig v",
                 "npix", "sat"))
print("-" * WIDTH)

# --- build the test matrix --------------------------------------------------
tests = []
for res, W, H in RES_LADDER:
    nm = RES_NAME[res]
    tests.append(("%s full sensor" % nm, res, (0, 0, sensor_w, sensor_h)))
    win = clamp_window(REF_U, REF_V, W, H, 0.0, 0.0)
    if win:
        tests.append(("%s 1:1 centred" % nm, res, win))
    win = clamp_window(REF_U, REF_V, W, H, 0.25, 0.25)
    if win:
        tests.append(("%s 1:1 offset" % nm, res, win))
    if 2 * W <= sensor_w and 2 * H <= sensor_h:
        win = clamp_window(REF_U, REF_V, 2 * W, 2 * H, 0.0, 0.0)
        if win:
            tests.append(("%s 2:1 downscale" % nm, res, win))

one_to_one = []

for name, res, win in tests:
    ok, req, got = apply_config(res, win[0], win[1], win[2], win[3])
    if not ok:
        print("%-20s %6s   SKIPPED (window %s rejected)" % (name, "-", str(req)))
        continue
    if got != req:
        print("    NOTE %s: requested %s, sensor gave %s"
              % (name, str(req), str(got)))
    st, al = start_str(got)
    xy = "(%d,%d)" % (got[0], got[1])
    # Where the reference sensor coord lands in THIS image. Pure inverse
    # of the affine map - no motion model, no filter, no time involved.
    px, py = sensor_to_image(REF_U, REF_V)
    m = measure(SAMPLES)
    if m is None:
        print("%-20s %6.3f %-11s %-10s %5s   NO MARKER (outside window?)"
              % (name, _geom[4], xy, st, al))
        continue
    iu, iv, mu, mv, sgu, sgv, npix, nsat, hits, clip = m
    du = mu - REF_U
    dv = mv - REF_V
    print(ROW_FMT % (name, _geom[4], xy, st, al, mu, mv, du, dv,
                     iu - px, iv - py, sgu, sgv, npix,
                     "?" if nsat < 0 else str(nsat))
          + ("  CLIPPED" if clip else "")
          + ("  SATURATED" if nsat > 0 else ""))
    if abs(_geom[4] - 1.0) < 0.01:
        one_to_one.append((name, mu, mv))

# --- verdict ----------------------------------------------------------------
print("-" * WIDTH)
if len(one_to_one) >= 2:
    us = [t[1] for t in one_to_one]
    vs = [t[2] for t in one_to_one]
    su = max(us) - min(us)
    sv = max(vs) - min(vs)
    spread = max(su, sv)
    print("SPREAD across %d ratio-1.000 configs: %.3f sensor px  (u %.3f, v %.3f)"
          % (len(one_to_one), spread, su, sv))
    if spread <= PASS_PX:
        print("PASS - the affine map is consistent across framesizes and windows.")
    else:
        bad = "u" if su > sv else "v"
        print("FAIL - %.3f px is too large, and it is almost all in %s."
              % (spread, bad))
        print("       A wrong sign or a wrong sensor_w/2 term in refresh_geom()")
        print("       breaks BOTH axes by tens of px. One axis off by ~1 px is")
        print("       the sensor not landing the window where it was asked to.")
        print("       GOT WINDOW cannot show that - the ioctl echoes the request")
        print("       back. Read the ALIGNMENT SWEEP below instead.")
else:
    print("INCONCLUSIVE - fewer than two ratio-1.000 configs saw the marker.")
print("=" * WIDTH)
print("The reference is measured at ratio 1.000, so du/dv are directly")
print("meaningful - not just their spread. SENSOR U/V is the same information")
print("without the subtraction: one stationary marker, one column of numbers")
print("that should not move.")
print("The '%s' row uses a window aimed from the REFINED position," % ref_name)
print("the reference used one aimed from the coarse full-sensor fix. If their")
print("START values differ, that row is NOT repeatability - it is two windows.")
print("Full-sensor rows sit at ratio ~%.1f and are EXPECTED to be ~%.1fx noisier."
      % (sensor_w / 640.0, sensor_w / 640.0))
print("npix should hold roughly constant across configs at ratio 1.000 and sat")
print("must stay 0. If npix moves with framesize, the effective exposure is")
print("moving with the window and the centroid shift is photometric, not")
print("geometric - re-run MODE=\"expose\" in kalmanFilter.py before believing")
print("anything above.")


# ---------------------------------------------------------------------------
# stage 4: alignment sweep
#
# The table above compares configurations that differ in several ways at once.
# This changes exactly one thing: it walks the readout window one sensor pixel
# at a time and asks whether the marker's SENSOR coordinate holds still.
#
# It must. Shift the window +1 px and the marker moves -1 image px; the map
# adds the window offset straight back and the sensor coordinate is unchanged.
# So RESID (measured sensor coord minus the reference) should be a flat line
# at zero, whatever the window offset is.
#
#   FLAT       the window lands where it was asked to. The map is right and
#              any error in the table above is somewhere else - look at npix.
#   STAIRCASE  the sensor is quantising the readout start onto an alignment
#              grid. TREAD is that grid in sensor px, RISE is how far the
#              image jumps each time it snaps. refresh_geom() is then being
#              handed a window the sensor never used, and no amount of
#              checking GOT WINDOW will show it, because the ioctl returns
#              the request rather than the register contents.
# ---------------------------------------------------------------------------


def bar(resid, span=3.0, cells=13):
    # One residual as a text plot. Middle cell is zero.
    mid = cells // 2
    row = ["-"] * cells
    row[mid] = "|"
    i = int(round(mid + resid / span * mid))
    if i < 0:
        row[0] = "<"
    elif i >= cells:
        row[cells - 1] = ">"
    else:
        row[i] = "*"
    return "[" + "".join(row) + "]"


def sweep_verdict(rows, axis):
    # rows: (step, resid, sig, img). The tell is d(img), not the residual.
    # An unquantised readout moves the image 1 px on every single step. A
    # quantised one holds it still, then jumps by the whole grid at once,
    # which makes RESID a sawtooth rather than a clean staircase.
    if len(rows) < 5:
        print("    INCONCLUSIVE - only %d usable steps." % len(rows))
        return
    r = [t[1] for t in rows]
    span = max(r) - min(r)
    noise = sum(t[2] for t in rows) / len(rows)
    movers = []
    stuck = 0
    for i in range(1, len(rows)):
        d = rows[i][3] - rows[i - 1][3]
        if abs(d) > 0.3:
            movers.append((rows[i][0], d))
        else:
            stuck += 1
    if stuck == 0 and span <= max(0.3, 8.0 * noise):
        print("    LINEAR - %d steps, image moved every step, residual span"
              % len(rows))
        print("    %.2f px against frame noise of %.2f. The window goes where"
              % (span, noise))
        print("    it is asked and the map cancels it. Nothing wrong here.")
        return
    if not movers:
        print("    STUCK - the image never moved across the whole %+d..%+d"
              % (-SWEEP_SPAN, SWEEP_SPAN))
        print("    sweep. Either the readout grid is coarser than the sweep -")
        print("    raise SWEEP_SPAN - or this window offset is being ignored")
        print("    outright. RESID span %.2f px." % span)
        return
    gaps = [movers[i][0] - movers[i - 1][0] for i in range(1, len(movers))]
    rise = sum(abs(m[1]) for m in movers) / len(movers)
    if not gaps:
        gs = "%.0f" % rise
    elif min(gaps) == max(gaps):
        gs = "%d" % min(gaps)
    else:
        gs = "%d-%d" % (min(gaps), max(gaps))
    print("    QUANTISED - the image held still for %d of %d steps, then moved"
          % (stuck, len(rows) - 1))
    print("    %.2f px every %s step(s). RESID sawtooths over %.2f px."
          % (rise, gs, span))
    print("    The readout start is on a %s px grid in %s, so refresh_geom() is"
          % (gs, axis))
    print("    being handed a window the sensor never used - and GOT WINDOW")
    print("    cannot show it, because the ioctl echoes the request back.")
    print("    Round window offsets to %s px before setting them (clamp_window)," % gs)
    print("    or measure the true start once and feed THAT to the map.")


def sweep_axis(res, W, H, axis):
    nm = RES_NAME[res]
    base = clamp_window(REF_U, REF_V, W, H, 0.0, 0.0)
    if base is None:
        print("")
        print("  %s %s sweep: no 1:1 window fits. SKIPPED" % (nm, axis))
        return
    lim = (sensor_w - W) // 2 if axis == "x" else (sensor_h - H) // 2
    nominal = base[0] if axis == "x" else base[1]
    ref_coord = REF_U if axis == "x" else REF_V
    print("")
    print("  %s 1:1 %dx%d - sweeping window %s over %+d..%+d sensor px"
          % (nm, W, H, axis, -SWEEP_SPAN, SWEEP_SPAN))
    print("  %5s %-12s %-11s %5s %9s %7s %9s %8s %6s %6s  %s"
          % ("STEP", "WIN x,y", "START", "C4,R4", "IMAGE", "d(img)",
             "SENSOR", "RESID", "sig " + axis, "npix", "-3 . 0 . +3"))
    rows = []
    prev = None
    for step in range(-SWEEP_SPAN, SWEEP_SPAN + 1):
        v = nominal + step
        if v < -lim or v > lim:
            print("  %5d  outside the clamp limit (+-%d). SKIPPED" % (step, lim))
            prev = None
            continue
        win = (v, base[1], W, H) if axis == "x" else (base[0], v, W, H)
        ok, req, got = apply_config(res, win[0], win[1], win[2], win[3])
        if not ok:
            prev = None
            continue
        if abs(_geom[4] - 1.0) > 0.01:
            print("  %5d  ratio %.3f, not 1:1. SKIPPED" % (step, _geom[4]))
            prev = None
            continue
        m = measure(SWEEP_SAMPLES)
        if m is None:
            print("  %5d  NO MARKER" % step)
            prev = None
            continue
        iu, iv, mu, mv, sgu, sgv, npix, nsat, hits, clip = m
        img = iu if axis == "x" else iv
        meas = mu if axis == "x" else mv
        sig = sgu if axis == "x" else sgv   # only the swept axis matters
        resid = meas - ref_coord
        st, al = start_str(got)
        dimg = "%+7.2f" % (img - prev) if prev is not None else "      -"
        prev = img
        print("  %5d %-12s %-11s %5s %9.2f %7s %9.2f %+8.2f %6.3f %6.0f  %s%s"
              % (step, "(%d,%d)" % (got[0], got[1]), st, al, img, dimg, meas,
                 resid, sig, npix, bar(resid),
                 "  SATURATED" if nsat > 0 else ""))
        rows.append((step, resid, sig, img))
    sweep_verdict(rows, axis)


if SWEEP:
    print("")
    print("=" * WIDTH)
    print("ALIGNMENT SWEEP - one window, moved one sensor pixel at a time")
    print("=" * WIDTH)
    print("RESID should be a flat line at 0.00 and d(img) should read -1.00 at")
    print("every step: the window moves right, the marker moves left, the map")
    print("cancels it. If d(img) sits at 0.00 for a few steps and then jumps 2")
    print("or 4 px at once, the sensor is aligning the readout start to a grid")
    print("and RESID sawtooths. That is the sensor, not the arithmetic.")
    _wh = {}
    for _res, _W, _H in RES_LADDER:
        _wh[_res] = (_W, _H)
    _steps = 2 * SWEEP_SPAN + 1
    print("%d configs x %d steps x %d frames = ~%d frames."
          % (2 * len(SWEEP_RES), _steps, SWEEP_SAMPLES + WINDOW_SETTLE
             + SETTLE_FRAMES,
             2 * len(SWEEP_RES) * _steps
             * (SWEEP_SAMPLES + WINDOW_SETTLE + SETTLE_FRAMES)))
    for _res in SWEEP_RES:
        if _res not in _wh:
            continue
        _W, _H = _wh[_res]
        sweep_axis(_res, _W, _H, "x")
        sweep_axis(_res, _W, _H, "y")
    print("=" * WIDTH)
