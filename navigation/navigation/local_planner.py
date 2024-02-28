#!/usr/bin/env python
"""
This is the ROS 2 node that handles the local planning for the JACART.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import numpy as np
import math
from navigation import pure_pursuit, cubic_spline_planner

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from nav_msgs.msg import Path
from navigation_interface.msg import (
    LocalPointsArray,
    VehicleState,
    Stop,
    WaypointsArray,
)

# We need to figure out how they are using VelAngle so we can use VelAngle/Vel
# Also the respecitive methods need to be ported over.
from motor_control_interface.msg import Vel, VelAngle

from std_msgs.msg import Float32, String, UInt64
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, Pose, Twist
from visualization_msgs.msg import Marker
import tf_transformations as tf


class LocalPlanner(rclpy.node.Node):
    """ROS2 node that handles local pathing and obstacle avoidance."""

    def __init__(self):
        super().__init__("local_planner")

        # driving constants
        self.METERS = 10.0
        self.SECONDS = 3.6

        # driving variables
        self.global_speed = self.METERS / self.SECONDS
        self.raw_speed = 0
        self.cur_speed = 0

        self.new_path = False
        self.path_valid = False
        self.local_points = []
        self.poll_sample = 0

        self.current_state = VehicleState()
        self.stop_requests = {}

        # ros variables
        self.cur_pose = Pose()  # current position in local coordinates
        self.cur_twist = Twist()  # current velocity (linear x value)

        ## subscribers
        # The points to use for a path coming from global planner
        self.global_path_sub = self.create_subscriber(
            LocalPointsArray, "/global_path", self.global_path_cb, 10
        )

        # The linear and angular velocity of the cart from NDT Matching
        self.twist_sub = self.create_subscriber(
            TwistStamped, "/estimate_twist", self.twist_cb, 10
        )

        # The position of the cart from NDT Matching
        self.pose_sub = self.create_subscriber(
            PoseStamped, "/ndt_pose", self.pose_cb, 10
        )

        # Current speed of the cart in M/s
        self.speed_sub = self.create_subscriber(
            Float32, "/estimated_vel_mps", self.speed_cb, 10
        )

        # Stop requests
        self.stop_sub = self.create_subscriber(Stop, "/stop", self.stop_cb, 10)

        # Change speed
        self.speed_req_sub = self.create_subscriber(
            Float32, "/speed", self.speed_req_cb, 10
        )

        ## Publisher
        # Share the current status of the vehicle's state
        self.vehicle_state_pub = self.create_publisher(
            VehicleState, "/vehicle_state", 10
        )

        # Send out speed and steering requests to motor endpoint
        # FIXME THIS PUBLISHER IS NEVER ACTUALLY BEING CALLED IN THE METHOD. THIS NEEDS TO BE FIXED.
        # self.motion_pub = self.create_publisher(VelAngle, "/nav_cmd", 10)

        # Publish points on the map in rviz
        self.points_pub = self.create_publisher(Path, "/points", 10)

        # Publish the cubic spline path in rviz
        self.path_pub = self.create_publisher(Path, "/path", 10)

        # Publish the next navigating point in the path
        self.target_pub = self.create_publisher(Marker, "/target_point", 10)

        # Publish the current requested steering angle
        self.target_twist_pub = self.create_publisher(Marker, "/target_twist", 10)

        # Publish status update for the server
        self.arrvied_pub = self.create_publisher(String, "/arrived", 10)

        # Steering angle PieChart display
        self.steering_pub = self.create_publisher(Float32, "/steering_angle", 10)

        # Publish the ETA
        self.eta_pub = self.create_publisher(UInt64, "/eta", 10)

        ## Timers
        # Calculate ETA
        self.eta_timer = self.create_timer(1, self.calc_eta)

        # Main loop
        self.timer = self.create_timer(0.5, self.timer_cb)

    def timer_cb(self):
        if self.new_path:
            self.path_valid = True
            self.new_path = False
            self.create_path()

    def twist_cb(self, msg):
        self.cur_twist = msg.twist

    def pose_cb(self, msg):
        self.cur_pose = msg.pose

    def stop_cb(self, msg):
        self.stop_requests[str[msg.sender_id.data].lower()] = [msg.stop, msg.distance]

    def speed_cb(self, msg):
        self.cur_speed = msg.data / self.SECONDS

    def vel_cb(self, msg):
        if msg.data < 1.0:
            self.cur_speed = 1.8  # Magic number :)
        else:
            self.poll_sample += 1
            self.raw_speed += msg.data
            if self.poll_sample >= 5:
                self.cur_speed = self.raw_speed / self.poll_sample
                self.raw_speed = 0
                self.poll_sample = 0

    def global_path_cb(self, msg):
        self.local_points = []
        for local_point in msg.localpoints:
            self.local_points.append(local_point.position)

        self.path_valid = False
        self.new_path = True


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = LocalPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
