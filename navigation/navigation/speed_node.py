#!/usr/bin/env python
"""
This node handles the calculation of speed for the golf cart.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import math

# Should change this in main to some ROS based time system but for now it works fine.
import time

# ROS based imports
import rclpy
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly


class SpeedNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("speed_node")

        # ROS2 publishers
        # The linear and angular velocity of the cart from NDT Matching
        self.twist_pub = self.create_publisher(TwistStamped, "/estimate_twist", 10)

        # ROS2 subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/pcl_pose", self.pose_cb, 10
        )

        self.prev_pose = None
        self.prev_time = 0
        self.speed_estimate = 0

        self.timer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        if self.speed_estimate != 0:
            self.twist_pub.publish(self.twist_estimate)

    def pose_cb(self, msg):
        """
        Simple callback for retrieving a pose. This callback also does the math to find the
        speed estimate based on the distance between two poses.
        """
        if self.prev_pose != None:
            # Change this to np later
            distance = math.sqrt(
                (msg.pose.pose.position.y - self.prev_pose.position.y) ** 2
                + (msg.pose.pose.position.x - self.prev_pose.position.x) ** 2
            )
            delta_time = time.time() - self.prev_time

            # smoothing

            self.speed_estimate = (
                self.speed_estimate * 0.8 + (distance / delta_time) * 0.2
            )
            self.twist_estimate = TwistStamped()
            self.twist_estimate.twist.linear.x = self.speed_estimate

        self.prev_pose = msg.pose.pose
        self.prev_time = time.time()

        # def log_header(self, msg):
        """Helper method to print noticeable log statements."""
        "Currently Commented out due to it drowning out other logs in the launch file"
        # self.get_logger().info("=" * 50)
        # self.get_logger().info(f"{msg}")
        # self.get_logger().info("=" * 50)


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = SpeedNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
