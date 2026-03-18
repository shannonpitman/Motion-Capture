# colourTrackUDP.py
# WiFi/UDP version of colourTrack.py for OpenMV RT1062 with built-in WiFi.
# Detects coloured markers and streams 21-byte binary packets over UDP.
#
# This work is licensed under the MIT license.
# Copyright (c) 2013-2025 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import sensor
import time
import struct
import network
import socket

CAM_ID = 2  # Unique ID per camera

# WiFi credentials
WIFI_SSID = "msg.network "  # "dlink-1C691"  # Replace with your WiFi network name
WIFI_KEY = "msg.t66Yu9"  # "0826583900"  # Replace with your WiFi password

# MATLAB host (the PC running UDPreceive.m)
HOST_IP = "137.158.122.116"  # "192.168.0.124"  # Replace with your MATLAB PC's IP address (powershell: ipconfig)
HOST_PORT = 7007  # Must match MATLAB listener port

#  COLOUR THRESHOLDS
sentinel = 65535

# LAB Colour Tracking Thresholds (L_min, L_max, A_min, A_max, B_min, B_max)
redThreshold = (15, 58, 25, 50, 15, 45)
greenThreshold = (10, 60, -50, -15, 10, 40)
blueThreshold = (11, 56, 6, 16, -41, -2)
yellowThreshold = (70, 100, -18, -1, 10, 60)

thresholds = [redThreshold, greenThreshold, blueThreshold, yellowThreshold]
num_features = len(thresholds)

# Blob detection parameters
PIXEL_THRESHOLD = 100
AREA_THRESHOLD = 100


#  WiFi CONNECTION
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

print("Connecting to WiFi '%s'..." % WIFI_SSID)
wlan.connect(WIFI_SSID, WIFI_KEY)

# Wait for connection (with timeout)
t0 = time.ticks_ms()
while not wlan.isconnected():
    if time.ticks_diff(time.ticks_ms(), t0) > 15000:
        raise OSError("WiFi connection timed out after 15 s")
    time.sleep_ms(100)

ifconfig = wlan.ifconfig()
print("Connected! IP: %s  Subnet: %s  Gateway: %s" % (ifconfig[0], ifconfig[1], ifconfig[2]))

#  UDP SOCKET
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = (HOST_IP, HOST_PORT)
print("Streaming UDP to %s:%d" % target)


#  CAMERA INIT
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

# ROI windowing
ROIsize = 100
frameW = sensor.width()
frameH = sensor.height()

lastUV = [None] * num_features
lostCount = [0] * num_features
maxLostFrames = 5

# Packet format: <B I 8H  (21 bytes)
#   B  : cam_id      (uint8)
#   I  : tick_ms     (uint32)
#   8H : u,v pairs   (uint16 x8: R_u, R_v, G_u, G_v, B_u, B_v, Y_u, Y_v)
PACKET_FMT = '<BI8H'


def pack_and_send(cam_id, tick_ms, features_uv):
    """Pack detection results into a 21-byte packet and send over UDP."""
    values = []
    for uv in features_uv:
        if uv is not None:
            values.append(uv[0])
            values.append(uv[1])
        else:
            values.append(sentinel)
            values.append(sentinel)
    packet = struct.pack(PACKET_FMT, cam_id, tick_ms, *values)
    sock.sendto(packet, target)


def make_roi(cx, cy, size):
    x = max(0, cx - size // 2)
    y = max(0, cy - size // 2)
    w = min(size, frameW - x)
    h = min(size, frameH - y)
    return (x, y, w, h)


#  MAIN LOOP
while True:
    # Check WiFi is still up; reconnect if dropped
    if not wlan.isconnected():
        print("WiFi lost — reconnecting...")
        wlan.connect(WIFI_SSID, WIFI_KEY)
        t0 = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), t0) > 15000:
                print("Reconnect failed — retrying next frame")
                break
            time.sleep_ms(100)
        if wlan.isconnected():
            print("Reconnected: %s" % wlan.ifconfig()[0])
        continue  # skip this frame

    clock.tick()
    img = sensor.snapshot()
    tick_ms = time.ticks_ms()

    # --- ROI logic (identical to USB version) ---
    use_roi = any(
        lastUV[i] is not None and lostCount[i] < maxLostFrames
        for i in range(num_features)
    )

    if use_roi:
        xs, ys, xe, ye = [], [], [], []
        for i in range(num_features):
            if lastUV[i] is not None:
                cx, cy = lastUV[i]
                roi = make_roi(cx, cy, ROIsize)
                xs.append(roi[0])
                ys.append(roi[1])
                xe.append(roi[0] + roi[2])
                ye.append(roi[1] + roi[3])
        roi = (min(xs), min(ys), max(xe) - min(xs), max(ye) - min(ys))
    else:
        roi = None

    if roi:
        blobs = img.find_blobs(
            thresholds, roi=roi,
            pixels_threshold=PIXEL_THRESHOLD,
            area_threshold=AREA_THRESHOLD, merge=True
        )
    else:
        blobs = img.find_blobs(
            thresholds,
            pixels_threshold=PIXEL_THRESHOLD,
            area_threshold=AREA_THRESHOLD, merge=True
        )

    featuresUV = [None] * num_features

    for blob in blobs:
        code = blob.code()
        for i in range(num_features):
            if code & (1 << i):
                if featuresUV[i] is None or blob.pixels() > featuresUV[i][2]:
                    featuresUV[i] = (blob.cx(), blob.cy(), blob.pixels())

    uv_out = [None] * num_features
    for i in range(num_features):
        if featuresUV[i] is not None:
            uv_out[i] = (featuresUV[i][0], featuresUV[i][1])
            lastUV[i] = uv_out[i]
            lostCount[i] = 0
        else:
            lostCount[i] += 1
            if lostCount[i] >= maxLostFrames:
                lastUV[i] = None

    pack_and_send(CAM_ID, tick_ms, uv_out)
