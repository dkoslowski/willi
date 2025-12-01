#! /usr/bin/env python3

#
# Run both motors at max speed for calibration purposes
# Direct hardware approach, no ROS or robot modules involved
#

import time
from adafruit_motorkit import MotorKit

kit = MotorKit(pwm_frequency = 50.0)
throttle = 1.0

try:
    print("Press ctrl+c to interrupt")
    print("Running both motors at full speed")
    kit.motor2.throttle = throttle
    kit.motor4.throttle = throttle
    # time.sleep(30)
    while True:
        pass
except KeyboardInterrupt:
    pass

kit.motor2.throttle = None
kit.motor4.throttle = None

print("\nBye!")

