#! /usr/bin/env python3

import time
from adafruit_motorkit import MotorKit

kit = MotorKit(pwm_frequency = 50.0)

class Willi():
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

    def __init__(self):
        self._kit = MotorKit(pwm_frequency = 50.0)
        self._motorL = self._kit.motor2
        self._motorR = self._kit.motor4

        self.speed = 0.0
        self.spin  = 0.0

    def _set_motor_speeds(self):
        self._motorL.throttle = self.speed
        self._motorR.throttle = self.speed


    def run(self):
        self.speed = 0.3
        self._set_motor_speeds()
        time.sleep(2)
        self.speed = 0.5
        self._set_motor_speeds()
        time.sleep(2)
        self.speed = 1.0
        self._set_motor_speeds()
        time.sleep(2)
        self.speed = 0.0
        self._set_motor_speeds()
        print('Bling!')

def main(args=None):
    willi = Willi()
    willi.run()

if __name__ == '__main__':
    main()
