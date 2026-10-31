"""Adapt the sandbox's stamped command to the simulator's Twist interface."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class StampedToTwist(Node):
    def __init__(self):
        super().__init__('stamped_to_twist')
        self.max_age = self.declare_parameter('max_age', 0.2).value
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_age must be finite and positive')
        self.last_stamp = None
        self.publisher = self.create_publisher(Twist, 'output', 10)
        self.create_subscription(TwistStamped, 'input', self.convert, 10)

    def convert(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if not 0 <= age <= self.max_age:
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.publisher.publish(message.twist)


def main(args=None):
    rclpy.init(args=args)
    node = StampedToTwist()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
