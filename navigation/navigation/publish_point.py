#!/usr/bin/env python3
"""
Simple script to publish a point to the 'new_point' topic for waypoint creation.

Usage: python3 publish_point.py <x> <y> [z]
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header


class PointPublisher(Node):
    def __init__(self):
        super().__init__('point_publisher')
        self.publisher = self.create_publisher(PointStamped, 'new_point', 10)

    def publish_point(self, x, y, z=0.0):
        msg = PointStamped()
        msg.header = Header()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)

        self.publisher.publish(msg)
        self.get_logger().info(f'Published point: ({x}, {y}, {z})')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: python3 publish_point.py <x> <y> [z]")
        return

    x = sys.argv[1]
    y = sys.argv[2]
    z = sys.argv[3] if len(sys.argv) > 3 else "0.0"

    node = PointPublisher()
    node.publish_point(x, y, z)

    # Keep the node alive briefly to ensure message is sent
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()