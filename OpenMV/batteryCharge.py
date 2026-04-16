import sensor

# Shut down the image sensor to minimize power draw
sensor.reset()
sensor.shutdown(True)
