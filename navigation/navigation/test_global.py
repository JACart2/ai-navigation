#!/usr/bin/env python
"""
This is the ROS 2 node that fakes localization data for the purpose of testing global planner.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import numpy as np

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from nav_msgs.msg import Path
from navigation_interface.msg import (
    LocalPointsArray,
    VehicleState,
    Stop,
)

from std_msgs.msg import Float32, String, UInt64, Header
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, Pose, PoseArray
from visualization_msgs.msg import Marker
import tf_transformations as tf


class GlobalTester(rclpy.node.Node):
    """ROS2 node that handles local pathing and obstacle avoidance."""

    def __init__(self):
        super().__init__("global_tester")

        self.pose_pub = self.create_publisher(PoseStamped, "/limited_pose", 10)
        self.vel_pub = self.create_publisher(Float32, "/estimated_vel_mps", 10)
        self.state_pub = self.create_publisher(VehicleState, "/vehicle_state", 10)

        self.rviz_path_pub = self.create_publisher(PoseArray, "/visual_path", 10)

        self.path_sub = self.create_subscription(
            LocalPointsArray, "/global_path", self.path_cb, 10
        )

        self.pose = PoseStamped()
        self.pose.pose.position.x = 45.0
        self.pose.pose.position.y = 60.0

        self.vel = Float32()
        self.vel.data = 0.0

        self.state = VehicleState()
        self.state.stopped = True
        self.state.reached_destination = False
        self.state.is_navigating = False

        self.timer = self.create_timer(0.5, self.timer_cb)

    def timer_cb(self):
        self.pose_pub.publish(self.pose)
        self.vel_pub.publish(self.vel)
        self.state_pub.publish(self.state)

    def path_cb(self, msg):
        # self.rviz_path_pub.publish(msg.localpoints)
        self.get_logger().info(f"{str(msg)}")


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = GlobalTester()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
