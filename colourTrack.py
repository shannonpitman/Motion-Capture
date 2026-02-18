# This work is licensed under the MIT license.
# Copyright (c) 2013-2025 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor
import time

# Colour Tracking Thresholds
redThreshold = (30, 48, 26, 56, 16, 48)
greenThreshold = (19, 62, -36, -27, -8, 14)

thresholds = [redThreshold, greenThreshold]
colors = [(255, 0, 0), (0, 255, 0)]

# Init Cam
sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.VGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=2000)  # Wait for settings take effect.
sensor.set_auto_gain(False)  # Disable auto gain for colour tracking
sensor.set_auto_whitebal(False)  # Disable auto white balance
clock = time.clock()  # Create a clock object to track the FPS.

while True:
    clock.tick()  # Update the FPS clock.
    img = sensor.snapshot()  # Take a picture and return the image.
    # to the IDE. The FPS should increase once disconnected.
    for i, threshold in enumerate(thresholds):
        blobs = img.find_blobs([threshold], pixels_threshold=100, area_threshold=100)
        for blob in blobs:
            # Draw rectangle with colour specfic visualisation
            img.draw_rectangle(blob.rect(), color=colors[i])
            img.draw_cross(blob.cx(), blob.cy(), colors=colors[i])
            print(f"Color{i} at: ({blob.cx()}, {blob.cy()})")
    print(clock.fps())  # Note: OpenMV Cam runs about half as fast when connected
