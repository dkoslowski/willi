#! /usr/bin/env python3

import time
from range_sensor import RangeSensor

print("Testing willi's distance sensor")

# Measure the distance every second
try:
    print("Press ctrl+c to interrupt")
    sensor = RangeSensor()
    while True:
        print('Distance measured: ', sensor.get_distance())
        time.sleep(1)

except Exception as e:
    print(e)

except KeyboardInterrupt:
    print('\nBye!')

