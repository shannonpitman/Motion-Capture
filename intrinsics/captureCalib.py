# captureCalib.py - Shannon Pitman
# Grabs calibration frames on the OpenMV RT1062 and writes them to the SD card.
#
# 1. Save to the cam (Tools -> Save Open Script to OpenMV Cam), set CAM_ID first.
# 2. Disconnect the IDE, power cycle. Blue = warming up, red = hold still,
#    green flash = frame saved. Move the board between flashes.
# 3. When the LEDs go solid green it is done. Replug the cam with the IDE
#    closed: the SD card mounts as a USB drive, drag /calib_camN off it.
#
# Lossless .pgm, VGA 1:1 on the sensor array, so the intrinsics come out in the
# same sensor pixels the tracker filters in.

import csi
import time
import os
from machine import LED

CAM_ID = 1
SHOTS = 40
INTERVAL_S = 3.0        # seconds between captures
WARMUP_S = 8.0          # time to walk to the board before the first shot
GAIN_DB = 0
EXPOSURE_US = 0         # 0 = auto exposure, otherwise fixed

OUT_DIR = "/sd/calib_cam%d" % CAM_ID

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(csi.VGA)
cam.auto_gain(False, gain_db=GAIN_DB)
if EXPOSURE_US > 0:
    cam.auto_exposure(False, exposure_us=EXPOSURE_US)
else:
    cam.auto_exposure(True)
cam.snapshot(time=2000)

try:
    red, green, blue = LED("LED_RED"), LED("LED_GREEN"), LED("LED_BLUE")
except Exception:
    red = green = blue = None


def led(which, on):
    if which is not None:
        which.on() if on else which.off()


if "sd" not in os.listdir("/"):
    OUT_DIR = "/calib_cam%d" % CAM_ID   # no SD card - internal flash, keep SHOTS low
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
        info = "  mean=%s max=%s" % (s.mean(), s.max())
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
