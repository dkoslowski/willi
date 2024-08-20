#! /usr/bin/env python3

import time
import wheel
from adafruit_motorkit import MotorKit

# ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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
class Willi(Node):

    def __init__(self):

        super().__init__('willi_node')

        self._kit = MotorKit(pwm_frequency = 50.0)
        self._wheelL = wheel.Wheel(motor = self._kit.motor2, d_mm = 65, max_rpm = 185.6, min_throttle = 0.2)
        self._wheelR = wheel.Wheel(motor = self._kit.motor4, d_mm = 65, max_rpm = 183.1, min_throttle = 0.2)
        self._max_speed = min(self._wheelL.max_speed(), self._wheelR.max_speed())
        self.speed = 0.0
        self.spin  = 0.0

        self._command_subscription = self.create_subscription(
            String,
            'command',
            self._command_callback,
            10)


    def _command_callback(self, msg):
        command = msg.data
        self.get_logger().info('Command received: "%s"' % command)
        if command == 'forward':
            self._forward()
        elif command == 'backward':
            self._backward()
        elif command == 'left':
            self._left()
        elif command == 'right':
            self._right()
        elif command == 'stop':
            self.stop()
        else:
            print('Unknown command, stopping instead')
            self.stop()

    def _set_speed(self, speedL, speedR):
        self._wheelL.set_speed(speedL)
        self._wheelR.set_speed(speedR)

    def _set_throttle(self, throttleL, throttleR):
        self._wheelL.set_throttle(throttleL)
        self._wheelR.set_throttle(throttleR)

    # get absolute and normalized speed for the robot
    def _normalize(self, speed):
        if speed == None:
            s = self._max_speed
        else:
            s = min(abs(speed), this._max_speed)
        return s

    def _forward(self, speed=None):
        s = self._normalize(speed)
        self._wheelL.set_speed(s)
        self._wheelR.set_speed(s)

    def _backward(self, speed=None):
        s = self._normalize(speed)
        self._wheelL.set_speed(-s)
        self._wheelR.set_speed(-s)
        
    def _left(self, speed=None):
        s = self._normalize(speed)
        self._wheelL.set_speed(-s)
        self._wheelR.set_speed(s)
        
    def _right(self, speed=None):
        s = self._normalize(speed)
        self._wheelL.set_speed(s)
        self._wheelR.set_speed(-s)
        
    def stop(self):
        self._wheelL.stop()
        self._wheelR.stop()

    def __del__(self):
        self.stop()

def main(args=None):

    rclpy.init(args=args)

    willi = Willi()
    print('Spinning.')
    rclpy.spin(willi)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
