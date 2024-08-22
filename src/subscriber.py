#! /usr/bin/env python3

# ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Range

import willi

class WilliSubscriber(Node):

    def __init__(self):

        super().__init__('willi_subscriber')

        self._command_subscription = self.create_subscription(
            String,
            'command',
            self._command_callback,
            10)

        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            2)

        self._joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            5)

        self._range_subscription = self.create_subscription(
            Range,
            'range',
            self._range_callback,
            5)

        self._willi = willi.Willi()

    def _command_callback(self, msg):
        command = msg.data
        self.get_logger().info('Command received: "%s"' % command)
        if command == 'forward':
            self._willi.set_velocity(speed=0.5)
        elif command == 'backward':
            self._willi.set_velocity(speed=-0.5)
        elif command == 'left':
            self._willi.set_velocity(turn=1.0)
        elif command == 'right':
            self._willi.set_velocity(turn=-1.0)
        elif command == 'stop':
            self._willi.stop()
        else:
            print('Unknown command, stopping instead')
            self._willi.stop()

    def _cmd_vel_callback(self, msg):
        speed = msg.linear.x
        turn = msg.angular.z
        self.get_logger().info(f'Speed by keyboard: {speed}, turn: {turn}')
        self._willi.set_velocity(speed, turn)

    def _joy_callback(self, msg):
        if msg.buttons[5] == 1:
            self.get_logger().info(f'Stop by joystick')
            self._willi.stop()
        else:
            turn  = msg.axes[0]
            speed = msg.axes[1]
            self.get_logger().info(f'Speed by joystick: {speed}, turn: {turn}')
            self._willi.set_velocity(speed, turn)

    def _range_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    subscriber = WilliSubscriber()

    print('Spinning.')
    rclpy.spin(subscriber)

    rclpy.shutdown()

if __name__ == '__main__':
    main()