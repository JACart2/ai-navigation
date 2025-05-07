#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class PoseBridge(Node):
    def __init__(self):
        super().__init__('pose_bridge')

        # Subscribe to LIO-SAM pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/lio_sam/mapping/pose',
            self.pose_callback,
            10
        )

        # Publish to /pcl_pose
        self.pcl_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/pcl_pose',
            10
        )

    def pose_callback(self, msg):
        bridged_msg = PoseWithCovarianceStamped()

        # Fill header
        bridged_msg.header = msg.header

        # Copy pose
        bridged_msg.pose.pose = msg.pose

        # Optionally fill covariance matrix with zeros (or small uncertainty if you want)
        bridged_msg.pose.covariance = [
            0.0 for _ in range(36)
        ]

        # Publish the bridged message
        self.pcl_pub.publish(bridged_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseBridge()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
