#! /usr/bin/env python3

import math
from adafruit_motorkit import MotorKit

# A whhel on a DC motor controlled by an Adafruit RPi motor HAT
class Wheel:

    def __init__(self, motor, d_mm, max_rpm, min_throttle):
        self._motor = motor
        self._max_speed = d_mm / 1000 * math.pi * max_rpm / 60
        self._min_throttle = min_throttle

    def __del__(self):
        self.stop()

    # set wheel throttle (-1.0..1.0)    
    def set_throttle(self, throttle):
        # sanitize the parameter
        if abs(throttle) < self._min_throttle:
            throttle = 0
        elif throttle > 1.0:
            throttle = 1.0
        elif throttle < -1.0:
            throttle = -1.0
        self._motor.throttle = throttle
        
    # set wheel speed (m/s)    
    def set_speed(self, speed):
        # # sanitize the parameter
        # if speed > self._max_speed:
        #     speed = self._max_speed
        # elif speed < -self._max_speed:
        #     speed = -self._max_speed
        self.set_throttle(speed/self._max_speed)

    # stop the wheel
    def stop(self):
        self._motor.throttle = None
