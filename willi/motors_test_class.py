#! /usr/bin/env python3

#
# Test motor related class methods
# Class appropach, no ROS involved
# willi ROS package must be built and activated previously
#

import time
from numpy import interp
import willi.robot

# maximal speed 0,617427676 m/s

print("Testing willi's motors")
willi = willi.robot.Willi()

try:
    print("Press ctrl+c to interrupt")

    for i in range(-10, 11):
        throttle = i / 10
        print("throttle:", throttle)
        willi.set_throttle(throttle, throttle)
        time.sleep(2)
    willi.stop()

    time.sleep(2)

    max_speed = willi.max_speed()
    for i in range(-10, 11):
        speed = interp(i, [-10, 10], [-max_speed, max_speed])
        print("speed m/s:", speed)
        willi.set_speed(speed, speed)
        time.sleep(2)
    willi.stop()

    print("Finished")

except KeyboardInterrupt:
    print("\nInterrpted")

