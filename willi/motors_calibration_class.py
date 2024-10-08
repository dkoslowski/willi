#! /usr/bin/env python3

#
# Motors calibration using class methods
# Class approach, no ROS involved
# willi ROS package must be built and activated previously
#

import time
from willi.robot import Willi

def calibrate_throttle():
    while True:
        s = input('Enter the throttle value (-1.0 to 1.0): ')
        try:
            throttle = float(s)
            print('Set throttle to ', throttle)
            willi.set_throttle(throttle, throttle)
        except ValueError:
            print('Not a float: ', s)

def calibrate_speed():
    while True:
        s = input('Enter the speed value in m/s (-1.0 to 1.0): ')
        try:
            speed = float(s)
            print('Set speed to ', speed)
            willi.set_speed(speed, speed)
        except ValueError:
            print('Not a float: ', s)

willi = Willi()
print("Calibrating willi's motors")

try:
    print("Press ctrl+c to interrupt")
    while True:
        inp = input('Calibrate [t]hrottle or [s]peed? ')
        if inp in ['t', 'T']:
            calibrate_throttle()
            break
        elif inp in ['s', 'S']:
            calibrate_speed()
            break

except KeyboardInterrupt:
    print("\nInterrpted")
