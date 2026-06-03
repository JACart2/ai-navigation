#!/usr/bin/env python3

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class InitialMapVizPublisher(Node):
    def __init__(self):
        super().__init__("publish_initial_map_viz")

        self.declare_parameter("map_path", "/maps/speedBoiMap_thinner.pcd")
        self.declare_parameter("topic", "/initial_map_viz")
        self.declare_parameter("frame_id", "map")

        self.map_path = self.get_parameter("map_path").value
        self.topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PointCloud2, self.topic, qos_profile)

        self.cloud = self._load_cloud(self.map_path)
        self.published = False
        self.timer = self.create_timer(0.5, self._publish_once)

        self.get_logger().info(
            f"Prepared {self.cloud.width} points for {self.topic} from {self.map_path}"
        )

    def _load_cloud(self, map_path: str) -> PointCloud2:
        with open(map_path, "r", encoding="utf-8") as map_file:
            while True:
                line = map_file.readline()
                if not line:
                    raise RuntimeError(f"Reached EOF while reading PCD header: {map_path}")
                if line.startswith("DATA"):
                    break

            data = np.loadtxt(map_file, dtype=np.float32)

        if data.ndim == 1:
            data = data.reshape(1, -1)

        if data.shape[1] < 4:
            raise RuntimeError(
                f"Expected at least 4 columns in PCD data, got {data.shape[1]} from {map_path}"
            )

        points = data[:, :4]
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        return point_cloud2.create_cloud(header, fields, points.tolist())

    def _publish_once(self):
        if self.published:
            return

        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.cloud)
        self.published = True
        self.get_logger().info(f"Published latched initial map viz on {self.topic}")


def main():
    rclpy.init(args=sys.argv)
    node = InitialMapVizPublisher()
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
