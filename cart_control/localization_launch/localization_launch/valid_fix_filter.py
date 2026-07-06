#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class ValidFixFilter(Node):
    def __init__(self):
        super().__init__("valid_fix_filter")

        self.declare_parameter("input_topic", "/fix")
        self.declare_parameter("output_topic", "/fix_valid")
        self.declare_parameter("min_status", int(NavSatStatus.STATUS_FIX))

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.min_status = int(self.get_parameter("min_status").value)

        self.pub = self.create_publisher(NavSatFix, output_topic, 10)
        self.sub = self.create_subscription(NavSatFix, input_topic, self.fix_cb, 10)

        self.last_warn_ns = 0

        self.get_logger().info(
            f"Filtering GPS fixes: {input_topic} -> {output_topic}, "
            f"requiring status >= {self.min_status}"
        )

    def fix_cb(self, msg: NavSatFix):
        valid_status = msg.status.status >= self.min_status
        valid_latlon = math.isfinite(msg.latitude) and math.isfinite(msg.longitude)

        if valid_status and valid_latlon:
            self.pub.publish(msg)
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warn_ns > 5_000_000_000:
            self.last_warn_ns = now_ns
            self.get_logger().warn(
                "Ignoring invalid GPS fix: "
                f"status={msg.status.status}, "
                f"lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ValidFixFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
