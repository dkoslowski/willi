#! /usr/bin/env python3

#
# HC-SR04 range sensor test
# Direct hardware approach, no ROS or robot modules involved
# willi ROS package must be built and activated previously
#

import time
import willi.sensor

print("Testing willi's distance sensor")
sensor = willi.sensor.Sensor()

# Measure the distance every second
try:
    print("Press ctrl+c to interrupt")
    while True:
        print('Distance measured: ', sensor.get_distance())
        time.sleep(1)

except Exception as e:
    print(e)

except KeyboardInterrupt:
    print('\nBye!')

