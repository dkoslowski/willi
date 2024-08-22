#! /usr/bin/env python3

import time
import wheel
from adafruit_motorkit import MotorKit

# # ROS 2
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist

class Willi():

    def __init__(self):

        # super().__init__('willi_node')

        self._wheel_base     = .132

        self._kit = MotorKit(pwm_frequency = 50.0)
        self._wheelL = wheel.Wheel(motor = self._kit.motor2,
                                   min_throttle = 0.3,
                                   max_throttle = 1.0,
                                   wheel_diameter = 0.065,
                                   min_rpm =  81.9,
                                   max_rpm = 185.6)
        self._wheelR = wheel.Wheel(motor = self._kit.motor4,
                                   min_throttle = 0.3,
                                   max_throttle = 1.0,
                                   wheel_diameter = 0.065,
                                   min_rpm =  84.6,
                                   max_rpm = 183.1)

        self._min_speed = max(self._wheelL.min_speed(), self._wheelR.min_speed())
        self._max_speed = min(self._wheelL.max_speed(), self._wheelR.max_speed())
        # min_speed: 0.2879269667015045, max_speed: 0.6231610827783154
        # print(f'min_speed: {self._min_speed}, max_speed: {self._max_speed}')

        self.speed = 0.0
        self.turn  = 0.0
    
    def min_speed(self):
        return self._min_speed

    def max_speed(self):
        return self._max_speed
    
    #     self._command_subscription = self.create_subscription(
    #         String,
    #         'command',
    #         self._command_callback,
    #         10)

    #     self._cmd_vel_subscription = self.create_subscription(
    #         Twist,
    #         'cmd_vel',
    #         self._cmd_vel_callback,
    #         2)

    # def _command_callback(self, msg):
    #     command = msg.data
    #     self.get_logger().info('Command received: "%s"' % command)
    #     if command == 'forward':
    #         self._forward()
    #     elif command == 'backward':
    #         self._backward()
    #     elif command == 'left':
    #         self._left()
    #     elif command == 'right':
    #         self._right()
    #     elif command == 'stop':
    #         self.stop()
    #     else:
    #         print('Unknown command, stopping instead')
    #         self.stop()

    # def _cmd_vel_callback(self, msg):
    #     self.speed = msg.linear.x
    #     self.turn = msg.angular.z
    #     self._set_motor_speeds()

    def _set_motor_speeds(self):

        # linear speed, m/s
        linear_speed = max(-self._max_speed, min(self._max_speed, self.speed))

        # angular speed, m/s
        angular_speed = max(-self._max_speed, min(self._max_speed, self.turn * self._wheel_base / 2))

        # if the composite speed exceeds the motor speed, clip the linear speed
        speedL = linear_speed - angular_speed
        speedR = linear_speed + angular_speed
        if speedR > self._max_speed:
            linear_speed =  self._max_speed - angular_speed
        elif speedR < -self._max_speed:
            linear_speed = -self._max_speed - angular_speed
        if speedL > self._max_speed:
            linear_speed =  self._max_speed + angular_speed
        elif speedL < -self._max_speed:
            linear_speed = -self._max_speed + angular_speed

        # set speed
        speedL = linear_speed - angular_speed
        speedR = linear_speed + angular_speed
        self._wheelL.set_speed(speedL)
        self._wheelR.set_speed(speedR)

    def set_speed(self, speedL, speedR):
        self._wheelL.set_speed(speedL)
        self._wheelR.set_speed(speedR)

    def set_throttle(self, throttleL, throttleR):
        self._wheelL.set_throttle(throttleL)
        self._wheelR.set_throttle(throttleR)

    def set_velocity(self, speed=0.0, turn=0.0):
        self.speed = speed
        self.turn = turn
        self._set_motor_speeds()

    # def forward(self, speed=0.5):
    #     self.speed = abs(speed)
    #     self.turn  = 0.0
    #     self._set_motor_speeds()

    # def backward(self, speed=0.5):
    #     self.speed = -abs(speed)
    #     self.turn  =  0.0
    #     self._set_motor_speeds()
        
    # def left(self, turn=1.0):
    #     self.speed =  0.0
    #     self.turn  =  abs(turn)
    #     self._set_motor_speeds()
        
    # def right(self, turn=1.0):
    #     self.speed =  0.0
    #     self.turn  = -abs(turn)
    #     self._set_motor_speeds()
        
    def stop(self):
        self._wheelL.stop()
        self._wheelR.stop()
        self.speed =  0.0
        self.turn  =  0.0

    def __del__(self):
        self.stop()
