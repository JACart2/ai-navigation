#!/usr/bin/env python
"""
This is the ROS 2 node that handles the local planning for the JACART.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import numpy as np
import math

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from std_msgs.msg import Float32


class LocalPlanner(rclpy.node.Node):
    """ROS2 node that handles local pathing and obstacle avoidance."""

    def __init__(self):
        super().__init__("local_planner")


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = LocalPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
