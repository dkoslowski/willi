#! /usr/bin/env python3

import time
import wheel
from adafruit_motorkit import MotorKit

class Willi():

    def __init__(self):

        # super().__init__('willi_node')

        self._wheel_base     = 0.132
        self._wheel_diameter = 0.065

        self._kit = MotorKit(pwm_frequency = 50.0)
        self._wheelL = wheel.Wheel(motor = self._kit.motor2,
                                   min_throttle = 0.3,
                                   max_throttle = 1.0,
                                   wheel_diameter = self._wheel_diameter,
                                   min_rpm =  81.9,
                                   max_rpm = 185.6)
        self._wheelR = wheel.Wheel(motor = self._kit.motor4,
                                   min_throttle = 0.3,
                                   max_throttle = 1.0,
                                   wheel_diameter = self._wheel_diameter,
                                   min_rpm =  84.6,
                                   max_rpm = 183.1)

        self._min_speed = max(self._wheelL.min_speed(), self._wheelR.min_speed())   # 0.2879269667015045 m/s
        self._max_speed = min(self._wheelL.max_speed(), self._wheelR.max_speed())   # 0.6231610827783154 m/s
        self._max_turn = 2.0 * self._max_speed / self._wheel_diameter               # 19.174187162409705 rad/s (too big???)
        # print(f'min_speed: {self._min_speed}, max_speed: {self._max_speed}, max_turn = {self._max_turn}')

        self.speed = 0.0
        self.turn  = 0.0
        self.close = 0.30  # start slowing down when this close
        self.tooclose = 0.10   # no forward motion when this close
        self.distance = 100.0
    
    def min_speed(self):
        return self._min_speed

    def max_speed(self):
        return self._max_speed
    
    def _set_motor_speeds(self):

        lin_speed = self.speed
        ang_speed = self.turn * self._wheel_base
        if lin_speed < 0:
            ang_speed = -ang_speed

        # Distance check
        if lin_speed > 0.0:
            governor = 1.0
            if self.distance < self.tooclose:
                governor = 0.0
            elif self.distance < self.close:
                governor = (self.distance - self.tooclose) / (self.close - self.tooclose)
            lin_speed *= governor

        speedL = lin_speed - ang_speed
        speedR = lin_speed + ang_speed

        max_speed = max([abs(speedL), abs(speedR)])
        if max_speed > self._max_speed:
            factor = self._max_speed / max_speed
            speedL *= factor
            speedR *= factor

        self._wheelL.set_speed(speedL)
        self._wheelR.set_speed(speedR)

    # def _set_motor_speeds(self):

    #     # linear speed, m/s
    #     linear_speed = max(-self._max_speed, min(self._max_speed, self.speed))

    #     # angular speed, m/s
    #     angular_speed = max(-self._max_speed, min(self._max_speed, self.turn * self._wheel_base / 2))

    #     # if the composite speed exceeds the motor speed, clip the linear speed
    #     speedL = linear_speed - angular_speed
    #     speedR = linear_speed + angular_speed
    #     if speedR > self._max_speed:
    #         linear_speed =  self._max_speed - angular_speed
    #     elif speedR < -self._max_speed:
    #         linear_speed = -self._max_speed - angular_speed
    #     if speedL > self._max_speed:
    #         linear_speed =  self._max_speed + angular_speed
    #     elif speedL < -self._max_speed:
    #         linear_speed = -self._max_speed + angular_speed

    #     # set speed
    #     speedL = linear_speed - angular_speed
    #     speedR = linear_speed + angular_speed
    #     self._wheelL.set_speed(speedL)
    #     self._wheelR.set_speed(speedR)

    def set_speed(self, speedL, speedR):
        self._wheelL.set_speed(speedL)
        self._wheelR.set_speed(speedR)

    def set_throttle(self, throttleL, throttleR):
        self._wheelL.set_throttle(throttleL)
        self._wheelR.set_throttle(throttleR)

    def set_velocity(self, speed=0.0, turn=0.0):
        self.speed = speed
        self.turn  = turn
        self._set_motor_speeds()

    def set_norm_velocity(self, speed_factor=0.0, turn_factor=0.0):
        self.speed = self._max_speed * speed_factor
        self.turn  = self._max_turn * turn_factor * 0.2 # adjustment for smooth turning steering
        self._set_motor_speeds()

    def set_distance(self, distance):
        self.distance = distance
        self._set_motor_speeds()

    def stop(self):
        self._wheelL.stop()
        self._wheelR.stop()
        self.speed =  0.0
        self.turn  =  0.0

    def __del__(self):
        self.stop()
