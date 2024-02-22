#!/usr/bin/env python
"""
This is the ROS 2 node that handles the global planning for the JACART.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import serial as sr
import numpy as np
import bitstruct
import math

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from motor_control_interface.msg import VelCurr, VelAnglePlanned
from std_msgs.msg import Bool, String


class GlobalPlanner(rclpy.node.Node):
    """ROS2 node that handles controlling the motor."""

    def __init__(self):
        super().__init__("global_planner")


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = GlobalPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
