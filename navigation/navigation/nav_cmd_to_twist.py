#!/usr/bin/env python3
"""Publish a bounded localization twist prediction from navigation commands."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from motor_control_interface.msg import VelAngle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class NavCmdToTwist(Node):
    """Bridge desired cart motion into lidar_localization's twist input."""

    def __init__(self):
        super().__init__("nav_cmd_to_twist")

        self.declare_parameter("wheel_base", 1.676)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("max_speed_mps", 3.0)
        self.declare_parameter("max_steering_deg", 40.0)

        self.wheel_base = self.get_parameter("wheel_base").value
        self.command_timeout_sec = self.get_parameter("command_timeout_sec").value
        self.max_speed_mps = self.get_parameter("max_speed_mps").value
        self.max_steering_deg = self.get_parameter("max_steering_deg").value

        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.latest_cmd: Optional[VelAngle] = None
        self.latest_cmd_time = None

        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, "/twist", 10)
        self.cmd_sub = self.create_subscription(VelAngle, "/nav_cmd", self.cmd_cb, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_cb)

    def cmd_cb(self, msg: VelAngle):
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def timer_cb(self):
        twist = TwistWithCovarianceStamped()
        now = self.get_clock().now()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = "base_link"

        command_is_fresh = (
            self.latest_cmd is not None
            and self.latest_cmd_time is not None
            and (now - self.latest_cmd_time).nanoseconds * 1e-9
            <= self.command_timeout_sec
        )

        if command_is_fresh:
            speed = max(0.0, min(float(self.latest_cmd.vel), self.max_speed_mps))
            steering_deg = max(
                -self.max_steering_deg,
                min(float(self.latest_cmd.angle), self.max_steering_deg),
            )
            steering_rad = math.radians(steering_deg)

            twist.twist.twist.linear.x = speed
            if abs(self.wheel_base) > 1e-6:
                twist.twist.twist.angular.z = speed / self.wheel_base * math.tan(
                    steering_rad
                )

        # High covariance: this is only a short-horizon seed, not authoritative odom.
        twist.twist.covariance[0] = 0.5
        twist.twist.covariance[7] = 1.0
        twist.twist.covariance[14] = 1.0
        twist.twist.covariance[21] = 1.0
        twist.twist.covariance[28] = 1.0
        twist.twist.covariance[35] = 0.25

        self.twist_pub.publish(twist)


def main():
    rclpy.init()
    node = NavCmdToTwist()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
