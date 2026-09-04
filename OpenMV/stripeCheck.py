# stripeCheck.py - Shannon Pitman
# Diagnose horizontal banding across the marker blob.
#
# The OV5640 is a ROLLING shutter. At 85 us exposure with a ~21 us row time
# only about 4 rows integrate at once, so anything modulating the light above
# a few kHz writes itself into the image as horizontal bands. At 300 us that
# averages over ~14 rows and hides.
#
# Two candidates, and they are told apart by whether the stripes MOVE:
#   temporal (supply ripple, PWM)  -> phase drifts frame to frame, and the
#                                     depth falls as exposure rises
#   sensor row artifact (FPN)      -> phase locked to the same rows, and the
#                                     depth is roughly constant with exposure
#
# Run it with the marker fitted and still. Prints a row profile you can read
# by eye plus the two numbers that decide it.

import csi
import time

EXPOSURES = [21, 42, 85, 170, 341, 683]   # us, on the ~21.3 us grid
FRAMES = 10
ROW_TIME_US = 21.3      # measured: exposure quantises to this. Full-frame
                        # value - a windowed readout may differ, so treat any
                        # frequency below as an order of magnitude, not exact.
GAIN_DB = 0
THRESH_LO = 80
SEARCH_RES = csi.VGA
MEASURE_W = 320
MEASURE_H = 240
PROFILE_COLS = 46       # width of the ASCII profile

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(SEARCH_RES)
cam.snapshot(time=1000)
cam.auto_gain(False, gain_db=GAIN_DB)
cam.auto_exposure(False, exposure_us=EXPOSURES[len(EXPOSURES) // 2])
cam.snapshot(time=2000)

_x0, _y0, sensor_w, sensor_h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
thresh = [(THRESH_LO, 255)]
print("sensor array %dx%d" % (sensor_w, sensor_h))


def set_exposure(us):
    cam.auto_exposure(False, exposure_us=us)
    for _ in range(4):
        cam.snapshot()          # flush frames exposed at the old setting
    try:
        return cam.exposure_us()
    except Exception:
        return us


def find_and_window():
    # Locate at search resolution, then drop to a 1:1 window. Banding must be
    # judged at ratio 1: the binned search view resamples rows and can invent
    # or hide stripes on its own.
    cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (0, 0, sensor_w, sensor_h))
    cam.snapshot()
    for _ in range(200):
        img = cam.snapshot()
        blobs = img.find_blobs(thresh, area_threshold=8, pixels_threshold=8,
                               merge=True)
        if blobs:
            b = max(blobs, key=lambda z: z.pixels)
            r = min(sensor_w / float(cam.width()), sensor_h / float(cam.height()))
            cx = (b.cxf - cam.width() * 0.5) * r + sensor_w / 2.0
            cy = (b.cyf - cam.height() * 0.5) * r + sensor_h / 2.0
            break
        time.sleep_ms(30)
    else:
        print("no marker found - fit one and rerun")
        return False
    cam.framesize(csi.QVGA)
    cam.snapshot()
    x = int(cx - sensor_w / 2.0)
    y = int(cy - sensor_h / 2.0)
    xl = (sensor_w - MEASURE_W) // 2
    yl = (sensor_h - MEASURE_H) // 2
    cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW,
              (max(-xl, min(xl, x)), max(-yl, min(yl, y)), MEASURE_W, MEASURE_H))
    cam.snapshot()
    return True


def row_profile(img, buf):
    # Sum each row across the blob's bounding box -> one number per sensor row.
    # merge=True here on purpose: the banding itself splits the blob, and this
    # has to measure the WHOLE disc, not one bright slice of it.
    W = img.width()
    H = img.height()
    blobs = img.find_blobs(thresh, area_threshold=8, pixels_threshold=8,
                           merge=True)
    if not blobs:
        return None, 0, 0
    b = max(blobs, key=lambda z: z.pixels)
    nsplit = len(img.find_blobs(thresh, area_threshold=8, pixels_threshold=8,
                                merge=False))
    prof = []
    y = max(0, b.y)
    ylim = min(H, b.y + b.h)
    x0 = max(0, b.x)
    x1 = min(W, b.x + b.w)
    peak = 0
    while y < ylim:
        base = y * W
        s = 0
        x = x0
        while x < x1:
            p = buf[base + x]
            s += p
            if p > peak:
                peak = p
            x += 1
        prof.append(s / float(x1 - x0))
        y += 1
    return prof, peak, nsplit


def depth_and_period(prof):
    # Modulation depth over the middle of the disc, avoiding the soft edges
    # where the profile falls off for geometric reasons rather than banding.
    n = len(prof)
    if n < 8:
        return 0.0, 0, 0
    a = n // 4
    b = n - n // 4
    mid = prof[a:b]
    hi = max(mid)
    lo = min(mid)
    depth = (hi - lo) / (hi + lo) if (hi + lo) > 0 else 0.0
    mean = sum(mid) / len(mid)
    ac = [v - mean for v in mid]
    best_lag = 0
    best_c = 0.0
    for lag in range(2, min(16, len(ac) // 2)):
        c = 0.0
        for i in range(len(ac) - lag):
            c += ac[i] * ac[i + lag]
        c /= (len(ac) - lag)
        if c > best_c:
            best_c = c
            best_lag = lag
    phase = ac.index(max(ac))          # row of a stripe crest, for drift
    return depth, best_lag, phase


def bar(v, vmax):
    n = int(PROFILE_COLS * v / vmax) if vmax > 0 else 0
    return "#" * n


if find_and_window():
    print("")
    print("%8s %8s %6s %6s %8s %8s %s"
          % ("req_us", "act_us", "peak", "blobs", "depth", "period", "phase per frame"))
    worst = None
    for e in EXPOSURES:
        act = set_exposure(e)
        depths = []
        phases = []
        peak_max = 0
        split_max = 0
        keep = None
        for _ in range(FRAMES):
            img = cam.snapshot()
            try:
                buf = img.bytearray()
            except Exception:
                buf = None
            if buf is None:
                continue
            prof, peak, nsplit = row_profile(img, buf)
            if prof is None or len(prof) < 8:
                continue
            d, per, ph = depth_and_period(prof)
            depths.append(d)
            phases.append(ph)
            if peak > peak_max:
                peak_max = peak
            if nsplit > split_max:
                split_max = nsplit
            if keep is None:
                keep = prof
        if not depths:
            print("%8d %8d %6s %6s %8s %8s  no blob above threshold"
                  % (e, act, "-", "-", "-", "-"))
            continue
        dm = sum(depths) / len(depths)
        _, per, _ = depth_and_period(keep)
        print("%8d %8d %6d %6d %8.3f %6d rw  %s"
              % (e, act, peak_max, split_max, dm, per,
                 " ".join("%d" % p for p in phases)))
        if worst is None or dm > worst[1]:
            worst = (act, dm, per, keep)

    if worst:
        act, dm, per, prof = worst
        print("")
        print("row profile at %d us (deepest banding), one frame:" % act)
        vmax = max(prof)
        for i, v in enumerate(prof):
            print("  %3d %6.1f |%s" % (i, v, bar(v, vmax)))
        if per:
            print("")
            print("  stripe period ~%d rows -> ~%.1f kHz at %.1f us/row"
                  % (per, 1000.0 / (per * ROW_TIME_US), ROW_TIME_US))
        print("")
        print("READ IT LIKE THIS")
        print("  phase numbers jump around frame to frame -> the LIGHT is")
        print("     modulating. Look at what supplies the 9 V: a switching")
        print("     regulator in pulse-skipping mode ripples at exactly these")
        print("     frequencies. A battery and a resistor cannot do this.")
        print("  phase numbers identical every frame -> a SENSOR row artifact,")
        print("     not your LED. Check it against any steady bright source.")
        print("  depth falling as exposure rises -> temporal, averaging out.")
        print("  depth flat across exposures     -> fixed pattern.")
