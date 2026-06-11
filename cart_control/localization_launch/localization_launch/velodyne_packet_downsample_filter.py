#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from velodyne_msgs.msg import VelodyneScan


class VelodynePacketDownsampleFilter(Node):
    def __init__(self):
        super().__init__("velodyne_packet_downsample_filter")

        # ROS topic to subscribe to for raw Velodyne packet scans.
        self.declare_parameter("input_topic", "/velodyne_packets")
        # ROS topic to publish the filtered Velodyne packet scans on.
        self.declare_parameter("output_topic", "/velodyne_packets_filtered")
        # Keep one packet every N packets. A value of 1 disables downsampling.
        self.declare_parameter("packet_stride", 1)

        self.packet_stride = max(1, int(self.get_parameter("packet_stride").value))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            VelodyneScan, self.get_parameter("output_topic").value, qos
        )
        self.subscription = self.create_subscription(
            VelodyneScan,
            self.get_parameter("input_topic").value,
            self._scan_callback,
            qos,
        )

        self.get_logger().info(
            f"Filtering Velodyne packets with packet_stride={self.packet_stride}"
        )

    def _scan_callback(self, scan: VelodyneScan):
        if self.packet_stride <= 1:
            self.publisher.publish(scan)
            return

        filtered_scan = VelodyneScan()
        filtered_scan.header = scan.header
        filtered_scan.packets = self._filter_packets(scan.packets)
        self.publisher.publish(filtered_scan)

    def _filter_packets(self, packets):
        return packets[:: self.packet_stride]


def main():
    rclpy.init()
    node = VelodynePacketDownsampleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
