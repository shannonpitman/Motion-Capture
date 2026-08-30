# ledCharacterise.py - Shannon Pitman
# LED / marker characterisation on the OpenMV RT1062 (csi API).
#
# This was MODE="led" in kalmanFilter.py. It is standalone on purpose: nothing
# here touches the tracker. No R, no CENTROID_SIGMA, no AREA_NOMINAL_SENSOR,
# no association, no filter state. It needs only the imaging layer:
#
#   exposure and gain, the blob threshold, a 1:1 readout window so sigma comes
#   out in real sensor pixels, and the intensity-weighted centroid.
#
# One pass per marker: exposure, peak, saturation, area, npix, blob count, w/h
# and centroid sigma, printed in LED_characterisation.xlsx column order. It
# loops forever - measure, remove the marker, fit the next one, it measures
# again. No reflashing between samples.
#
# HOLD CONSTANT across every row or the comparison is void:
#   distance, angle, ambient light, camera position, GAIN_DB, THRESH_LO,
#   REFINE_FLOOR, AREA_MIN_SENSOR.
#
# EXPOSURE_US is deliberately NOT constant. Peak is the controlled variable:
# retarget the exposure per row until peak lands 200-230, and record what it
# took. The measurement block prints the next exposure to try.

import csi
import time
import math

# --- what you change between rows ------------------------------------------
EXPOSURE_US = 386     # retarget per row - see "next exposure" in the output
SAMPLES = 200         # frames per measurement

# --- hold these constant ----------------------------------------------------
GAIN_DB = 0
THRESH_LO = 80        # blob threshold
REFINE_FLOOR = 50     # background subtracted before intensity weighting
AREA_MIN_SENSOR = 120.0   # smallest blob accepted, in sensor px
SAT_LEVEL = 250       # a pixel at or above this is clipped
TARGET_PEAK = 210.0   # centre of the 200-230 band

# --- sensor / geometry ------------------------------------------------------
SEARCHING_RESOLUTION = csi.VGA
MEASURE_RES = csi.QVGA
MEASURE_W = 320       # readout window is set to exactly the framesize, so the
MEASURE_H = 240       # sensor never downscales and ratio stays 1.000
WINDOW_SETTLE = 1     # frames to flush after a window change
MIN_AREA_IMG = 4
REFINE_PAD = 2
REFINE_MAX_PX = 3000

# Exposure is quantised to one sensor row time. Measured 2026-08 on this
# camera: every requested value landed on a multiple of ~21.3 us, and the
# floor is one step. Used only to round the suggested next exposure.
EXPOSURE_QUANTUM_US = 21.3

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(SEARCHING_RESOLUTION)
cam.snapshot(time=1000)
cam.auto_gain(False, gain_db=GAIN_DB)
cam.auto_exposure(False, exposure_us=EXPOSURE_US)
cam.snapshot(time=3000)

_x0, _y0, sensor_w, sensor_h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
print("sensor array %dx%d | full-readout ratio: VGA %.2f  QVGA %.2f"
      % (sensor_w, sensor_h, sensor_w / 640.0, sensor_w / 320.0))

thresh = [(THRESH_LO, 255)]
current_res = SEARCHING_RESOLUTION

# Cached per frame: (W, H, w, h, ratio, offx, offy)
# sensor_x = (image_x - W/2) * ratio + offx
_geom = None


def refresh_geom():
    # Call after ANY framesize or readout-window change.
    global _geom
    x, y, w, h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
    W = cam.width()
    H = cam.height()
    ratio = min(w / float(W), h / float(H))
    offx = (w - (W * ratio)) / 2.0 + x + (sensor_w / 2.0)
    offy = (h - (H * ratio)) / 2.0 + y + (sensor_h / 2.0)
    _geom = (W, H, w, h, ratio, offx, offy)
    return _geom


def image_to_sensor(cx, cy):
    W, H, w, h, ratio, offx, offy = _geom
    return ((cx - W * 0.5) * ratio + offx,
            (cy - H * 0.5) * ratio + offy)


def reset_to_search():
    # Full sensor at the search framesize. Ratio > 1 here, which is fine for
    # LOCATING the marker but never for measuring it.
    global current_res
    if current_res != SEARCHING_RESOLUTION:
        cam.framesize(SEARCHING_RESOLUTION)
        current_res = SEARCHING_RESOLUTION
        cam.snapshot()  # discard the first frame after a resolution change
    cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (0, 0, sensor_w, sensor_h))
    for _ in range(WINDOW_SETTLE):
        cam.snapshot()
    refresh_geom()


def center_measure_window(cx, cy):
    # Readout window set to exactly the framesize and centred on the marker,
    # so ratio comes out 1.000 and sigma is in real sensor pixels. This is the
    # MODE="led" path of kalmanFilter.py's center_window, specialised: one
    # fixed size, no ladder, no caching.
    global current_res
    w, h = MEASURE_W, MEASURE_H
    x = int(cx - (sensor_w / 2.0))     # window centre, offset from sensor centre
    y = int(cy - (sensor_h / 2.0))
    xlim = (sensor_w - w) // 2
    ylim = (sensor_h - h) // 2
    xc = max(-xlim, min(xlim, x))
    yc = max(-ylim, min(ylim, y))
    try:
        if current_res != MEASURE_RES:
            cam.framesize(MEASURE_RES)
            current_res = MEASURE_RES
            cam.snapshot()  # discard the first frame after a resolution change
        cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (xc, yc, w, h))
        for _ in range(WINDOW_SETTLE):
            cam.snapshot()
    except OSError as e:
        print("  window %dx%d @ (%d,%d) rejected: %s" % (w, h, xc, yc, e))
        reset_to_search()
        return False
    refresh_geom()
    return True


def get_buffer(img):
    # Raw pixel bytes, or None if unavailable. Row y starts at y * stride.
    try:
        return img.bytearray()
    except (AttributeError, MemoryError):
        return None


def refine_centroid(buf, stride, W, H, bx, by, bw, bh):
    # Intensity-weighted centroid over the blob box + REFINE_PAD.
    # Identical to kalmanFilter.py - the sigma this reports must be the sigma
    # the tracker will see, so this one must not drift from it.
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
        return None  # really big blobs -> use the binary blob centroid
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
    # +0.5 matches the pixel-centre convention find_blobs uses
    return (sx / sw + 0.5, sy / sw + 0.5, npix, nsat, clipped)


def area_threshold(ratio):
    return max(MIN_AREA_IMG, int(AREA_MIN_SENSOR / (ratio * ratio)))


def wait_for_marker():
    # Locate at search resolution. Returns sensor coordinates.
    reset_to_search()
    at = area_threshold(_geom[4])
    waited = 0
    while True:
        img = cam.snapshot()
        blobs = img.find_blobs(thresh, area_threshold=at,
                               pixels_threshold=at, merge=False)
        if blobs:
            b = max(blobs, key=lambda z: z.pixels)
            return image_to_sensor(b.cxf, b.cyf)
        waited += 1
        if waited % 60 == 0:
            print("  waiting for a marker...")
        time.sleep_ms(30)


def wait_for_removal():
    reset_to_search()
    at = area_threshold(_geom[4])
    clear = 0
    while clear < 20:
        img = cam.snapshot()
        blobs = img.find_blobs(thresh, area_threshold=at,
                               pixels_threshold=at, merge=False)
        clear = clear + 1 if not blobs else 0
        time.sleep_ms(20)


def actual_exposure():
    try:
        return cam.exposure_us()
    except Exception:
        return None


def measure(n):
    # One measurement at ratio 1. Returns a dict, or None to retry.
    W, H, _w, _h, ratio, _ox, _oy = _geom
    if abs(ratio - 1.0) > 0.01:
        # A sigma measured at ratio 4 is ~16x too large - refuse it rather
        # than print a number that looks plausible and is not.
        print("  ratio=%.3f, not 1.000 - window was rejected. Retrying." % ratio)
        time.sleep_ms(500)
        return None
    at = area_threshold(ratio)

    peak_max = 0        # brightest pixel seen in ANY frame (extreme value)
    peak_sum = 0        # sum of per-frame peaks -> mean, the stable number
    nsat_max = 0
    nsat_sum = 0
    frames_sat = 0      # how many frames had ANY clipped pixel
    blobs_max = 0       # >1 means the four LEDs are not blending
    wh_sum = 0.0
    area_sum = 0.0
    npix_sum = 0
    su = sv = suu = svv = 0.0
    u0 = None
    v0 = 0.0
    k = 0
    tries = 0
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
        if len(blobs) > blobs_max:
            blobs_max = len(blobs)
        b = max(blobs, key=lambda z: z.pixels)

        # peak and saturated-pixel count inside the blob box. fpeak is THIS
        # frame's peak; averaging it over frames gives a stable number, where
        # the max over 200 frames x ~1000 px is an extreme value that a single
        # noise excursion can drag to 255.
        fpeak = 0
        nsat = 0
        y = max(0, b.y)
        ylim = min(H, b.y + b.h)
        while y < ylim:
            row = y * W
            x = max(0, b.x)
            xlim = min(W, b.x + b.w)
            while x < xlim:
                px = buf[row + x]
                if px > fpeak:
                    fpeak = px
                if px >= SAT_LEVEL:
                    nsat += 1
                x += 1
            y += 1
        peak_sum += fpeak
        if fpeak > peak_max:
            peak_max = fpeak
        nsat_sum += nsat
        if nsat > nsat_max:
            nsat_max = nsat
        if nsat > 0:
            frames_sat += 1

        r = refine_centroid(buf, W, W, H, b.x, b.y, b.w, b.h)
        if r is None:
            continue
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
        area_sum += b.pixels * ratio * ratio
        wh_sum += (b.w / float(b.h)) if b.h else 0.0
        k += 1

    if k < 20:
        print("  too few samples (%d) - marker too dim or moved. Retrying." % k)
        return None

    mu = su / k
    mv = sv / k
    sgu = math.sqrt(max(0.0, suu / k - mu * mu))
    sgv = math.sqrt(max(0.0, svv / k - mv * mv))
    return dict(k=k, ratio=ratio,
                peak_mean=peak_sum / float(k), peak_max=peak_max,
                nsat_mean=nsat_sum / float(k), nsat_max=nsat_max,
                frames_sat=frames_sat, sat_frac=frames_sat / float(k),
                area=area_sum / k, npix=npix_sum / float(k),
                blobs_max=blobs_max, wh=wh_sum / k,
                sigma_u=sgu, sigma_v=sgv,
                sigma_rms=math.sqrt(0.5 * (sgu * sgu + sgv * sgv)))


def report(idx, m, exp_used):
    if m["sat_frac"] > 0.05:
        verdict = "SATURATED - lower exposure"
    elif m["peak_mean"] < 180.0:
        verdict = "dim - raise exposure"
    elif m["peak_mean"] > 230.0:
        verdict = "hot - lower exposure"
    else:
        verdict = "ok, in band"

    print("")
    print("=== MEASUREMENT %d ===  (ratio=%.3f, %d frames)" % (idx, m["ratio"], m["k"]))
    print("  Exposure     %5d us   %s" % (exp_used, "" if exp_used == EXPOSURE_US
                                          else "(requested %d)" % EXPOSURE_US))
    print("  Peak mean    %5.1f      %s" % (m["peak_mean"], verdict))
    print("  Peak max     %5d      (single brightest px in any frame)" % m["peak_max"])
    print("  Sat px mean  %5.1f      max %d in one frame" % (m["nsat_mean"], m["nsat_max"]))
    print("  Sat frames   %5d / %d  (%.1f%%)" % (m["frames_sat"], m["k"], 100.0 * m["sat_frac"]))
    print("  Sensor area  %5.0f" % m["area"])
    print("  npix         %5.0f" % m["npix"])
    print("  blobs max    %5d      %s" % (m["blobs_max"],
          "<- NOT BLENDING, the LEDs are resolving separately" if m["blobs_max"] > 1 else ""))
    print("  w/h mean     %5.2f      %s" % (m["wh"],
          "<- LUMPY" if (m["wh"] < 0.9 or m["wh"] > 1.1) else ""))
    print("  sigma_u     %7.4f" % m["sigma_u"])
    print("  sigma_v     %7.4f" % m["sigma_v"])
    print("  sigma_RMS   %7.4f" % m["sigma_rms"])
    if m["sigma_u"] > 0:
        ratio_vu = m["sigma_v"] / m["sigma_u"]
        print("  sig_v/sig_u %7.1f      %s" % (ratio_vu,
              "<- anisotropic, that is a systematic not sensor noise" if ratio_vu > 3.0 else ""))

    # Peak is linear in exposure while unclipped, so one step lands the band.
    if m["sat_frac"] > 0.0:
        print("  next exposure: reduce the light, not the exposure - the peak is")
        print("                 clipped so this cannot be scaled. Add series R,")
        print("                 or move to the real operating range.")
    elif m["peak_mean"] > 0:
        want = EXPOSURE_QUANTUM_US * round(exp_used * TARGET_PEAK
                                           / m["peak_mean"] / EXPOSURE_QUANTUM_US)
        print("  next exposure for peak %d: %d us%s"
              % (TARGET_PEAK, want, "  (already in band)"
                 if 200.0 <= m["peak_mean"] <= 230.0 else ""))

    if 0.0 < m["sat_frac"] <= 0.05:
        print("  NOTE: a few frames clipped. Usable, but you are close to the")
        print("        top. Drop exposure ~10% if you can.")
    print("  A_Bench E-H:  %d  %.0f  %.1f  %d"
          % (exp_used, m["peak_mean"], m["nsat_mean"], m["frames_sat"]))
    print("  A_Bench J-M:  %.0f  %.0f  %d  %.2f"
          % (m["area"], m["npix"], m["blobs_max"], m["wh"]))
    print("  A_Bench O-P:  %.4f  %.4f" % (m["sigma_u"], m["sigma_v"]))
    print("  -> remove the marker to arm the next measurement")


def main(n=SAMPLES):
    print("LED CHARACTERISATION")
    print("  exposure=%d us  gain=%d dB  thresh=%d  refine_floor=%d  samples=%d"
          % (EXPOSURE_US, GAIN_DB, THRESH_LO, REFINE_FLOOR, n))
    act = actual_exposure()
    if act is None:
        print("  (sensor exposure readback not available on this firmware)")
    else:
        print("  sensor reports exposure_us = %d (requested %d)" % (act, EXPOSURE_US))
    print("  Fit a marker. Remove it after each result to arm the next.")
    idx = 0
    while True:
        loc = wait_for_marker()
        if not center_measure_window(loc[0], loc[1]):
            continue
        m = measure(n)
        if m is None:
            continue
        idx += 1
        act = actual_exposure()
        report(idx, m, act if act is not None else EXPOSURE_US)
        wait_for_removal()


main()
