#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray


MESSAGE_TYPES = {
    "sensor_msgs/msg/PointCloud2": PointCloud2,
    "visualization_msgs/msg/MarkerArray": MarkerArray,
}


class WaitForSubscriber(Node):
    def __init__(self):
        super().__init__("wait_for_rviz_subscriber")

        self.declare_parameter("topic", "/initial_map")
        self.declare_parameter("message_type", "sensor_msgs/msg/PointCloud2")
        self.declare_parameter("poll_period_s", 0.5)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        message_type = (
            self.get_parameter("message_type").get_parameter_value().string_value
        )
        poll_period_s = (
            self.get_parameter("poll_period_s").get_parameter_value().double_value
        )

        if message_type not in MESSAGE_TYPES:
            raise ValueError(
                f"Unsupported message_type '{message_type}'. "
                f"Supported values: {', '.join(sorted(MESSAGE_TYPES))}"
            )

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(MESSAGE_TYPES[message_type], topic, qos_profile)
        self.topic = topic
        self.timer = self.create_timer(poll_period_s, self.timer_cb)

        self.get_logger().info(
            f"Waiting for RViz subscriber on {self.topic} before allowing publish"
        )

    def timer_cb(self):
        if self.publisher.get_subscription_count() == 0:
            return

        self.get_logger().info(f"RViz subscriber detected on {self.topic}")
        self.destroy_timer(self.timer)
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init(args=sys.argv)
    node = WaitForSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
