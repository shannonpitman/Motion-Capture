# This work is licensed under the MIT license.
# Copyright (c) 2013-2025 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import sensor
import time
import struct
import sys

CAM_ID = 2  # unique ID per camera: change per cam
sentinel = 65535

# LAB Colour Tracking Thresholds (L_min, L_max, A_min, A_max, B_min, B_max) a: green-red  b: blue-yellow
redThreshold = (15, 58, 25, 50, 15, 45)
greenThreshold = (10, 60, -50, -15, 10, 40)
blueThreshold = (11, 56, -5, 16, -50, -2)
yellowThreshold = (70, 100, -18, -1, 10, 60)

thresholds = [redThreshold, greenThreshold, blueThreshold, yellowThreshold]
num_features = len(thresholds)

# Blob detection parameters
PIXEL_THRESHOLD = 100  # Minimum pixel count
AREA_THRESHOLD = 100  # Minimum bounding box area

# Init Cam
sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA, (320 x240) VGA = (640x480)
sensor.skip_frames(time=2000)  # Wait for settings take effect
sensor.skip_frames(time=2000)  # Wait for settings take effect.
sensor.set_auto_gain(False)  # Disable auto gain for colour tracking
sensor.set_auto_whitebal(False)  # Disable auto white balance
clock = time.clock()  # Create a clock object to track the FPS.

# ROI windowing for blob detection
ROIsize = 100  # pixels
frameW = sensor.width()
frameH = sensor.height()

lastUV = [None]*num_features
lostCount = [0]*num_features
maxLostFrames = 5

# Packet format: <B I 8H  (21 bytes total)
#   B  : cam_id      (uint8)
#   I  : tick_ms     (uint32, ticks_ms at frame capture)
#   8H : u,v pairs   (uint16 x8: R_u, R_v, G_u, G_v, B_u, B_v, Y_u, Y_v)
PACKET_FMT = '<BI8H'


def pack_and_send(cam_id, tick_ms, features_uv):
    """
    Pack detection results into a 21-byte packet and send over USB.

    Parameters:
        cam_id: int, camera identifier (1-12)
        tick_ms: int, time.ticks_ms() at frame capture
        features_uv: list of (u, v) tuples or None for each feature [R, G, B, Y]
    """

    values = []
    for uv in features_uv:
        if uv is not None:
            values.append(uv[0])
            values.append(uv[1])
        else:
            values.append(sentinel)
            values.append(sentinel)
    packet = struct.pack(PACKET_FMT, cam_id, tick_ms, *values)
    sys.stdout.buffer.write(packet)


def make_roi(cx, cy, size):
    x = max(0, cx-size//2)
    y = max(0, cy-size//2)
    w = min(size, frameW-x)
    h = min(size, frameH-y)
    return (x, y, w, h)


# Main
while True:
    clock.tick()
    img = sensor.snapshot()  # trigger image capture
    tick_ms = time.ticks_ms()

    # Detect each colour feature
    use_roi = any(lastUV[i] is not None and lostCount[i] < maxLostFrames for i in range(num_features))
    if use_roi:
        # Union ROI over all active colours
        xs, ys, xe, ye = [], [], [], []
        for i in range(num_features):
            if lastUV[i] is not None:
                cx, cy = lastUV[i]
                roi = make_roi(cx, cy, ROIsize)
                xs.append(roi[0])
                ys.append(roi[1])
                xe.append(roi[0] + roi[2])
                ye.append(roi[1] + roi[3])
        roi = (min(xs), min(ys), max(xe)-min(xs), max(ye)-min(ys))
    else:
        roi = None  # full frame
    if roi:
        blobs = img.find_blobs(thresholds, roi=roi, pixels_threshold=PIXEL_THRESHOLD, area_threshold=AREA_THRESHOLD, merge=True)

    else:
        blobs = img.find_blobs(thresholds, pixels_threshold=PIXEL_THRESHOLD, area_threshold=AREA_THRESHOLD, merge=True)

    featuresUV = [None]*num_features

    for blob in blobs:
        code = blob.code()
        for i in range(num_features):
            if code & (1 << i):  # Blobs matches threshold i
                if featuresUV[i] is None or blob.pixels() > featuresUV[i][2]:
                    featuresUV[i] = (blob.cx(), blob.cy(), blob.pixels())
    uv_out = [None]*num_features
    for i in range(num_features):
        if featuresUV[i] is not None:
            uv_out[i] = (featuresUV[i][0], featuresUV[i][1])
            lastUV[i] = uv_out[i]
            lostCount[i] = 0
        else:
            lostCount[i] += 1
            if lostCount[i] >= maxLostFrames:
                lastUV[i] = None  # Reset ROI for this colour

    # Send packet every frame
    pack_and_send(CAM_ID, tick_ms, uv_out)
