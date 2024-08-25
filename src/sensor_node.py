#! /usr/bin/env python3

# ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Range

from range_sensor import RangeSensor

class SensorNode(Node):

    def __init__(self, name='sensor_node', frequency=10):

        # super().__init__('sensor_node')

        # self.sensor = RangeSensor()
        """
        Parameters
        ----------
        name: str
            The node name that will be used for this robot; defaults
            to "wheelie"
        pinTrigger : int
            The RaspPi GPIO pin that goes high for 10us to trigger the
            ultrasonic sensor
        pinEcho : int
            The RaspPi GPIO pin that goes high for the duration of when
            the sound is sent until it's received back
        frequency : int
            The frequency in Hz for how often measurements are requested
        """

        super().__init__(name)

        self._frequency = frequency
        self._distance_sensor = RangeSensor()
        self._distance_publisher = self.create_publisher(Range, "range", 5)

        self._range_msg = Range()
        self._range_msg.radiation_type = Range.ULTRASOUND
        self._range_msg.field_of_view = self._distance_sensor.angle
        self._range_msg.min_range = self._distance_sensor.min_range
        self._range_msg.max_range = self._distance_sensor.max_range

        self.start()

    def _distance_callback (self):
        self.distance = self._distance_sensor.get_distance()
        self._range_msg.range = self.distance
        self._distance_publisher.publish(self._range_msg)

    def stop (self):
        self.destroy_timer(self._distance_timer)

    def start (self):
        self._distance_timer = self.create_timer (1.0 / self._frequency, self._distance_callback)

def main(args=None):

    try:
        print("Press ctrl+c to interrupt")
        rclpy.init(args=args)
        node = SensorNode()
        print('Spinning.')
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('\nBye!')

    # rclpy.shutdown()

if __name__ == '__main__':
    main()