#! /usr/bin/env python3

import time
import wheel
from adafruit_motorkit import MotorKit

# maximal speed 0,617427676 m/s

'''
A ROS2 node for a 2-wheeled robot

Attributes
----------
speed : float
    Speed along the X axis in meters per second; positive is
    forward and negative is backward
spin : float
    Rotation about the pivot point in radians per second; positive
    is clockwise when viewed from above (right spin)
'''
class Willi():

    def __init__(self):
        self._kit = MotorKit(pwm_frequency = 50.0)
        # d=67mm @ 175 RPM (max throttle)
        self._wheelL = wheel.Wheel(motor = self._kit.motor2, d_mm = 67, max_rpm = 176, min_throttle = 0.2)
        self._wheelR = wheel.Wheel(motor = self._kit.motor4, d_mm = 67, max_rpm = 176, min_throttle = 0.2)
        self.speed = 0.0
        self.spin  = 0.0

    def _set_speed(self, speedL, speedR):
        self._wheelL.set_speed(speedL)
        self._wheelR.set_speed(speedR)

    def _set_throttle(self, throttleL, throttleR):
        self._wheelL.set_throttle(throttleL)
        self._wheelR.set_throttle(throttleR)

    def stop(self):
        self._wheelL.stop()
        self._wheelR.stop()

    def __del__(self):
        self.stop()

    def run(self):
        try:
            print("Press ctrl+c to interrupt")
            while True:
                time.sleep(1)
                print('.', end='', flush=True)
        except KeyboardInterrupt:
            print("\nBye!")


def main(args=None):
    willi = Willi()
    willi.run()

if __name__ == '__main__':
    main()
