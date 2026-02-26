# This work is licensed under the MIT license.
# Copyright (c) 2013-2025 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import sensor
import time
from machine import RTC
import struct
import sys

CAM_ID = 1  # unique ID per camera
sentinel = 65535

# Colour Tracking Thresholds
redThreshold = (30, 48, 26, 56, 16, 48)
greenThreshold = (19, 62, -36, -27, -8, 14)
blueThreshold = (33, 74, 85, 117, 90, 140)
yellowThreshold = (82, 156, 69, 154, 41, 74)

thresholds = [redThreshold, greenThreshold, blueThreshold, yellowThreshold]
num_features = len(thresholds)

# Blob detection parameters
PIXEL_THRESHOLD = 100  # Minimum pixel count
AREA_THRESHOLD = 100  # Minimum bounding box area

# Init Cam
sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA, (320 x240) VGA = (640x480)
sensor.skip_frames(time=2000)  # Wait for settings take effect.
sensor.set_auto_gain(False)  # Disable auto gain for colour tracking
sensor.set_auto_whitebal(False)  # Disable auto white balance
clock = time.clock()  # Create a clock object to track the FPS.

rtc = RTC()  # initialises the RTC time and date -> will keep the time until power is completely lost by the system
rtc.datetime((2026, 2, 25, 3, 15, 45, 0, 0))  # Format: (year, month, day, weekday, hour, minute, second, subsecond)
# weekday: 0=Mon...6=Sun, subsecond: hardware-dependent countdown
print(rtc.datetime())

# Record ticks at startup to compute sub-second offset
_last_rtc_second = -1
_ticks_at_second = time.ticks_ms()


# ROI windowing for blob detection
ROIsize = 100  # pixels
frameW = sensor.width()
frameH = sensor.height()

numColours = len(thresholds)
lastUV = [None]*numColours
lostCount = [0]*numColours
maxLostFrames = 5

def get_timestamp():
    """
    Get RTC hour, minute, second plus sub-second milliseconds.

    rtc.datetime() returns:
        (year, month, day, weekday, hour, minute, second, subsecond)
         [0]    [1]   [2]   [3]    [4]   [5]     [6]      [7]

    The RTC subsecond field is a hardware countdown counter -> resolution
    varies by board. time.ticks_ms() interpolates reliable milliseconds within
    the current second instead.

    Returns: (hour, minute, second, subsec_ms)
    """
    global _last_rtc_second, _ticks_at_second

    dt = rtc.datetime()
    hour = dt[4]
    minute = dt[5]
    second = dt[6]

    current_ticks = time.ticks_ms()

    # Detect second rollover to resync ticks
    if second != _last_rtc_second:
        _last_rtc_second = second
        _ticks_at_second = current_ticks

    # Milliseconds elapsed since last whole second
    subsec_ms = time.ticks_diff(current_ticks, _ticks_at_second) % 1000

    return (hour, minute, second, subsec_ms)


def find_best_blob(blobs):
    """Return the largest blob by pixel count, or None if no blobs found."""
    if not blobs:
        return None
    return max(blobs, key=lambda b: b.pixels())


def pack_and_send(cam_id, timestamp, features_uv):
    """
    Pack detection results into a 26-byte uint16 packet and send over USB.

    Parameters:
        cam_id: int, camera identifier (1-12)
        timestamp: tuple (hour, minute, second, subsec_ms)
        features_uv: list of (u, v) tuples or None for each feature [R, G, B, Y]
    """
    hour, minute, second, subsec_ms = timestamp

    values = [cam_id, hour, minute, second, subsec_ms]
    for uv in features_uv:
        if uv is not None:
            values.append(uv[0])
            values.append(uv[1])
        else:
            values.append(sentinel)
            values.append(sentinel)
    # Packet format (little endian uint16):
    # [0] cam_id
    # [1] hour
    # [2] minute
    # [3] second
    # [4] subsec_ms
    # [5-6]  R (u,v)
    # [7-8]  G (u,v)
    # [9-10] B (u,v)
    # [11-12] Y (u,v)
    packet = struct.pack('<13H', *values)  # Little-endian, 13 x uint16
    sys.stdout.buffer.write(packet)

def make_roi(cx, cy, size):
    x = max(0, cx-size//2)
    y = max(0, cy-size//2)
    w = min(size, frameW - x)
    h = min(size, frameH-y)
    return (x, y, w, h)

# ---- Main Loop ----
while True:
    clock.tick()
    img = sensor.snapshot()  # trigger image capture
    timestamp = get_timestamp()

    # Detect each colour feature
    use_roi = any(lastUV[i] is not None and lostCount[i] < maxLostFrames for i in range(numColours))
    if use_roi:
        # Union ROI over all active colours
        xs, ys, xe, ye = [], [], [], []
        for i in range(numColours):
            if lastUV[i] is not None:
                cx, cy = lastUV[i]
                roi = make_roi(cx, cy, ROIsize)
                xs.append(roi[0])
                ys.append(roi[1])
                xe.append(roi[0] + roi[2])
                ye.append(roi[1] + roi[3])

        roi = (min(xs), min(ys),
               max(xe) - min(xs),
               max(ye) - min(ys))
    else:
        roi = None  # full frame

    blobs = img.find_blobs(thresholds, roi = roi, pixels_threshold=PIXEL_THRESHOLD, area_threshold=AREA_THRESHOLD, merge = True)
    featuresUV = [None]*numColours
    best = find_best_blob(blobs)

    for b in range(numColours):
        if best[i]:
            uv = (best[i].cx(), best[i].cy())
            featuresUV.append(uv)
            lastUV[i] = uv
        else:
            lostCount += 1
            if lostCount:
                lastUV[i] = None

    # Send packet every frame
    pack_and_send(CAM_ID, timestamp, featuresUV)
