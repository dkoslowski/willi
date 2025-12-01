import math
from adafruit_motorkit import MotorKit
from numpy import interp

#
# A wheel on a DC motor controlled by Adafruit RPi motor HAT
#
class Wheel:

    def __init__(self, motor, min_throttle, max_throttle, wheel_diameter, min_rpm, max_rpm):
        self._motor = motor
        self._min_throttle = min_throttle
        self._max_throttle = max_throttle
        self._min_speed = wheel_diameter * math.pi * min_rpm / 60
        self._max_speed = wheel_diameter * math.pi * max_rpm / 60
        self.stop()

    #
    # set wheel throttle (-1.0..1.0)
    #
    def set_throttle(self, throttle):
        if throttle > 1:
            throttle = 1
        elif throttle < -1:
            throttle = -1

        # print(f'Set throttle to {throttle}')
        self._motor.throttle = throttle
        
    #
    # set wheel speed (m/s)
    #
    def set_speed(self, speed):
        self.set_throttle(speed/self._max_speed)

    #
    # maximal speed of this wheel (m/s)
    def max_speed(self):
        return self._max_speed

    #
    # minimal speed of this wheel (m/s)
    #
    def min_speed(self):
        return self._min_speed

    #
    # stop the wheel
    #
    def stop(self):
        self._motor.throttle = None

    def __del__(self):
        self.stop()

