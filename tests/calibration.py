#! /usr/bin/env python3

import time
from adafruit_motorkit import MotorKit

'''
Spins wheels until interrupted
'''

kit = MotorKit(pwm_frequency = 50.0)
throttle = 1.0

try:
    print("Press ctrl+c to interrupt")
    kit.motor2.throttle = throttle
    kit.motor4.throttle = throttle
    # time.sleep(30)
    while True:
        pass
except KeyboardInterrupt:
    pass

kit.motor2.throttle = None
kit.motor4.throttle = None

print("Bye!")

