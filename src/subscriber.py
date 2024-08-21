#! /usr/bin/env python3

# ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

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
        
        self._willi = willi.Willi()

    def _command_callback(self, msg):
        command = msg.data
        self.get_logger().info('Command received: "%s"' % command)
        if command == 'forward':
            self._willi.forward()
        elif command == 'backward':
            self._willi.backward()
        elif command == 'left':
            self._willi.left()
        elif command == 'right':
            self._willi.right()
        elif command == 'stop':
            self._willi.stop()
        else:
            print('Unknown command, stopping instead')
            self.stop()

    def _cmd_vel_callback(self, msg):
        speed = msg.linear.x
        spin = msg.angular.z
        self.get_logger().info(f'Speed: {speed}, spin: {spin}')
        self._willi.set_velocity(speed, spin)

def main(args=None):
    rclpy.init(args=args)

    subscriber = WilliSubscriber()

    print('Spinning.')
    rclpy.spin(subscriber)

    rclpy.shutdown()

if __name__ == '__main__':
    main()