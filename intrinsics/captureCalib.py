# captureCalib.py - Shannon Pitman
# Grabs calibration frames on the OpenMV RT1062 and writes them to the SD card.
#
# 1. Save to the cam (Tools -> Save Open Script to OpenMV Cam), set CAM_ID first.
# 2. Disconnect the IDE, power cycle. Blue = warming up, red = hold still,
#    green flash = frame saved. Move the board between flashes.
# 3. When the LEDs go solid green it is done. Replug the cam with the IDE
#    closed: the SD card mounts as a USB drive, drag /calib_camN off it.
#
# RESOLUTION - read this before changing it.
#   FULL_RES = True captures WQXGA2 2592x1944 at readout ratio 1.000, i.e. the
#   raw sensor array. The intrinsics then come out in SENSOR pixels, which is
#   the coordinate frame kalmanFilter.py reports centroids in - no rescaling.
#   FULL_RES = False captures VGA 640x480, which is the FULL SENSOR DOWNSCALED
#   by 4.050 (it is NOT a 1:1 crop). Intrinsics are then in 640x480 pixels and
#   fx, fy, cx, cy must be multiplied by 4.050 before use with the tracker.
#   k and p are in normalised coordinates and do not scale.
# A full-res frame is 5.04 MB, so 40 shots need ~200 MB. SD card required.

import csi
import time
import os
from machine import LED

CAM_ID = 2
FULL_RES = True         # True = 2592x1944 sensor pixels (see note above)
SHOTS = 40
INTERVAL_S = 3.0        # seconds between captures
WARMUP_S = 8.0          # time to walk to the board before the first shot

# Exposure and gain. Long exposures blur a handheld board, and blur biases
# corners INWARD - a systematic error, not noise. Trade exposure for gain:
# 12 dB buys a 4.2x shorter exposure for the same brightness. Measured on cam2
# with one IR torch: 8000 us at 12 dB ~= 33800 us at 0 dB (mean 73, max 198).
# Auto exposure is NOT usable here - it pins at the frame-rate ceiling
# (21450 us at VGA) and still underexposes. Set it explicitly.
GAIN_DB = 12
EXPOSURE_US = 8000      # 0 = auto (not recommended - see above)

OUT_DIR = "/sd/calib_cam%d" % CAM_ID

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(csi.WQXGA2 if FULL_RES else csi.VGA)
cam.auto_gain(False, gain_db=GAIN_DB)
if EXPOSURE_US > 0:
    cam.auto_exposure(False, exposure_us=EXPOSURE_US)
else:
    cam.auto_exposure(True)
cam.snapshot(time=2000)

_x0, _y0, _sw, _sh = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
_img = cam.snapshot()
_ratio = min(_sw / float(_img.width()), _sh / float(_img.height()))
print("capture %dx%d | readout %dx%d | ratio %.3f" %
      (_img.width(), _img.height(), _sw, _sh, _ratio))
if abs(_ratio - 1.0) > 0.01:
    print("NOTE: ratio is %.3f, so intrinsics will be in DOWNSCALED pixels." % _ratio)
    print("      Multiply fx, fy, cx, cy by %.3f before use with the tracker." % _ratio)

try:
    red, green, blue = LED("LED_RED"), LED("LED_GREEN"), LED("LED_BLUE")
except Exception:
    red = green = blue = None


def led(which, on):
    if which is not None:
        which.on() if on else which.off()


has_sd = "sd" in os.listdir("/")
if not has_sd:
    if FULL_RES:
        raise SystemExit("No SD card. A full-res run needs ~200 MB - "
                         "insert an SD card or set FULL_RES = False.")
    OUT_DIR = "/calib_cam%d" % CAM_ID   # internal flash - keep SHOTS low
try:
    os.mkdir(OUT_DIR)
except OSError:
    pass

led(blue, True)
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < WARMUP_S * 1000:
    cam.snapshot()
led(blue, False)

n = 0
while n < SHOTS:
    led(red, True)
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < INTERVAL_S * 1000:
        img = cam.snapshot()
    led(red, False)

    n += 1
    path = "%s/img_%03d.pgm" % (OUT_DIR, n)
    img.save(path)
    try:
        s = img.get_statistics()
        mean = s.mean() if callable(getattr(s, "mean", None)) else s.mean
        mx = s.max() if callable(getattr(s, "max", None)) else s.max
        info = "  mean=%s max=%s" % (mean, mx)
    except Exception:
        info = ""
    print("%d/%d %s%s" % (n, SHOTS, path, info))

    led(green, True)
    time.sleep_ms(250)
    led(green, False)

print("done - %d frames in %s" % (n, OUT_DIR))
led(green, True)
while True:
    time.sleep_ms(500)
