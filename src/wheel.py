#! /usr/bin/env python3

import math
from adafruit_motorkit import MotorKit
from numpy import interp

# A whhel on a DC motor controlled by an Adafruit RPi motor HAT
class Wheel:

    def __init__(self, motor, min_throttle, max_throttle, wheel_diameter, min_rpm, max_rpm):
        self._motor = motor
        self._min_throttle = min_throttle
        self._max_throttle = max_throttle
        self._min_speed = wheel_diameter * math.pi * min_rpm / 60
        self._max_speed = wheel_diameter * math.pi * max_rpm / 60
        self.stop()

    def __del__(self):
        self.stop()

    # set wheel throttle (-1.0..1.0)    
    def set_throttle(self, throttle):
        if throttle > 1:
            throttle = 1
        elif throttle < -1:
            throttle = -1

        # sanitize the parameter
        # if throttle > 0:
            # if throttle < self._min_throttle:
            #     throttle = self._min_throttle
            # elif throttle > self._max_throttle:
            #     throttle = self._max_throttle
        # elif throttle < 0:
            # if throttle > -self._min_throttle:
            #     throttle = -self._min_throttle
            # elif throttle < -self._max_throttle:
            #     throttle = -self._max_throttle

        print(f'Set throttle to {throttle}')
        self._motor.throttle = throttle
        
    # set wheel speed (m/s)    
    def set_speed(self, speed):
        # # sanitize the parameter
        # throttle = 0
        # if speed > 0:
        #     if speed < self._min_speed:
        #         speed = self._min_speed
        #     elif speed > self._max_speed:
        #         speed = self._max_speed
        #     throttle = interp(speed, [self._min_speed, self._max_speed], [self._min_throttle, self._max_throttle])
        # elif speed < 0:
        #     if speed > -self._min_speed:
        #         speed = -self._min_speed
        #     elif speed < -self._max_speed:
        #         speed = -self._max_speed
        #     throttle = interp(speed, [-self._max_speed, -self._min_speed], [-self._max_throttle, -self._min_throttle])

        # self.set_throttle(throttle)
        self.set_throttle(speed/self._max_speed)

    # get the maximal speed of this wheel
    def max_speed(self):
        return self._max_speed

    # get the minimal speed of this wheel
    def min_speed(self):
        return self._min_speed

    # stop the wheel
    def stop(self):
        self._motor.throttle = None
