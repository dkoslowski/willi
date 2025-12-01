import time
from adafruit_motorkit import MotorKit

from willi.wheel import Wheel

#
# Robot himself
#
class Willi():

    def __init__(self):

        self._wheel_base     = 0.132
        self._wheel_diameter = 0.065

        self._kit = MotorKit(pwm_frequency = 50.0)
        self._wheelL = Wheel(
            motor = self._kit.motor2,
            min_throttle = 0.3,
            max_throttle = 1.0,
            wheel_diameter = self._wheel_diameter,
            min_rpm =  81.9,
            max_rpm = 185.6)
        self._wheelR = Wheel(
            motor = self._kit.motor4,
            min_throttle = 0.3,
            max_throttle = 1.0,
            wheel_diameter = self._wheel_diameter,
            min_rpm =  84.6,
            max_rpm = 183.1)

        self._min_speed = max(self._wheelL.min_speed(), self._wheelR.min_speed())   # 0.2879269667015045 m/s
        self._max_speed = min(self._wheelL.max_speed(), self._wheelR.max_speed())   # 0.6231610827783154 m/s
        self._max_turn = 2.0 * self._max_speed / self._wheel_diameter               # 19.174187162409705 rad/s (too big???)
        # print(f'min_speed: {self._min_speed}, max_speed: {self._max_speed}, max_turn = {self._max_turn}')

        self.speed    =   0.0
        self.turn     =   0.0
        self.close    =   0.30 # start slowing down when this close
        self.tooclose =   0.10 # no forward motion when this close
        self.distance = 100.0
    
    #
    # Robot's minimal possible speed until motors stall (m/s)
    #
    def min_speed(self):
        return self._min_speed

    #
    # Robots maximal speed at full throttle (m/s)
    #
    def max_speed(self):
        return self._max_speed
    
    #
    # Update motors speed
    #
    def _update_motors_speed(self):

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

    #
    # Set motors' speed directly (m/s)
    #
    def set_speed(self, speedL, speedR):
        self._wheelL.set_speed(speedL)
        self._wheelR.set_speed(speedR)

    #
    # Set motors' throttle directly (-1.0..1.0)
    #
    def set_throttle(self, throttleL, throttleR):
        self._wheelL.set_throttle(throttleL)
        self._wheelR.set_throttle(throttleR)

    #
    # Set robot's velocity (linear m/s, turn rad/s)
    #
    def set_velocity(self, speed=0.0, turn=0.0):
        self.speed = speed
        self.turn  = turn
        self._update_motors_speed()

    #
    # Set robot's mormal velocity (linear, rotational -1.0..1.0)
    #
    def set_norm_velocity(self, speed_factor=0.0, turn_factor=0.0):
        self.speed = self._max_speed * speed_factor
        self.turn  = self._max_turn * turn_factor * 0.2 # adjustment for smooth turning steering
        self._update_motors_speed()

    #
    # Set obstacle distance
    #
    def set_distance(self, distance):
        self.distance = distance
        self._update_motors_speed()

    #
    # Stop the robot immediately
    #
    def stop(self):
        self._wheelL.stop()
        self._wheelR.stop()
        self.speed =  0.0
        self.turn  =  0.0

    def __del__(self):
        self.stop()
