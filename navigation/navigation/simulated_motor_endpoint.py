#!/usr/bin/env python
"""
This is a ROS2 node that simulates motion rather than sending messages to the arduino controller.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import serial as sr
import numpy as np
import bitstruct
import math

from navigation import steering_position_calc

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from tf_transformations import quaternion_from_euler

from motor_control_interface.msg import VelAngle
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from navigation_interface.msg import LocalPointsArray
from visualization_msgs.msg import MarkerArray, Marker


class SimulatedMotor(rclpy.node.Node):
    """ROS2 node that handles controlling the motor."""

    def __init__(self):
        super().__init__("motor_simulator")

        # Class constants
        self.NODE_RATE = 10

        self.vel = 0.0
        self.angle = 0.0

        self.x = 82.23206329345703
        self.y = 132.16149291992187

        self.new_vel = False

        # Creation of some simple subscribers/publishers/timers
        self.planned_motion_subscriber = self.create_subscription(
            VelAngle, "/nav_cmd", self.vel_angle_planned_callback, 10
        )
        # self.path_sub = self.create_subscription(
        #     LocalPointsArray, "/global_path", self.path_cb, 10
        # )

        self.pose_pub = self.create_publisher(PoseStamped, "/limited_pose", 10)
        self.vel_pub = self.create_publisher(Float32, "/estimated_vel_mps", 10)

        self.timer = self.create_timer(1.0 / self.NODE_RATE, self.timer_callback)

    def vel_angle_planned_callback(self, planned_vel_angle):
        """
        Callback method to get the target velocity and angle.
        This is achieved by using the subscription we created in init to recieved a
        message (planned_vel_angle) and set the appropriate fields to make the cart drive/turn.
        """

        print("IM getting a vel angle")
        self.vel = planned_vel_angle.vel
        self.angle = planned_vel_angle.angle

        if self.vel < 0:
            # indicates an obstacle
            self.vel = 0

        self.new_vel = True

    def timer_callback(self):
        """Main loop timer for updating motor's instructions."""

        # Check if we have received a target yet
        if self.new_vel:
            self.calculate_endpoint()
        self.get_logger().info(
            f"x:{self.x}, y:{self.y}, vel:{self.vel}, angle:{self.angle}"
        )
        self.prev_time = time.time()

        return

    def calculate_endpoint(self):
        """The endpoint for processing and sending instructions to the arduino controller."""

        cur_time = time.time()
        self.x, self.y, phi = steering_position_calc.calc_new_pos(
            cur_time - self.prev_time, self.x, self.y, self.vel, self.angle
        )

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        x, y, z, w = quaternion_from_euler(0.0, 0.0, phi)
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        self.pose_pub.publish(pose)

        vel = Float32()
        vel.data = self.vel
        self.vel_pub.publish(vel)

    # def path_cb(self, msg):
    #     # This array is used to delete the preexisting markerarrays before publishing
    #     delarr = MarkerArray()
    #     delmark = Marker()
    #     delmark.action = 3
    #     delarr.markers.append(delmark)
    #     self.rviz_path_pub.publish(delarr)

    #     arr = MarkerArray()
    #     id = 0
    #     for p in msg.localpoints:
    #         temp = Marker()
    #         temp.pose = p
    #         temp.header.frame_id = "world"
    #         temp.id = id
    #         id += 1
    #         temp.scale.x = 1.0
    #         temp.scale.y = 1.0
    #         temp.scale.z = 1.0
    #         temp.color.r = 0.0
    #         temp.color.g = 0.0
    #         temp.color.b = 1.0
    #         temp.color.a = 1.0
    #         temp.type = 2
    #         temp.action = 0
    #         arr.markers.append(temp)
    #     arr.markers[0].color.g = 50.0
    #     arr.markers[-1].color.r = 50.0
    #     self.rviz_path_pub.publish(arr)


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = SimulatedMotor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
