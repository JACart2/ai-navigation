#!/usr/bin/env python3

import argparse
import csv
import math
import re
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray


def read_fit_value(text, name):
    match = re.search(
        rf"^{re.escape(name)}:\s*([-+0-9.eE]+)\s*$",
        text,
        flags=re.MULTILINE,
    )

    if not match:
        raise RuntimeError(f"Missing {name} in transform YAML")

    return float(match.group(1))


def make_sphere(
    frame_id,
    stamp,
    namespace,
    marker_id,
    x,
    y,
    z,
    diameter,
    red,
    green,
    blue,
):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0

    marker.scale.x = diameter
    marker.scale.y = diameter
    marker.scale.z = diameter

    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = 1.0

    return marker


class PreviewPublisher(Node):
    def __init__(self, csv_path, fit_path, topic, frame_id):
        super().__init__("alignment_transform_preview_publisher")

        self.frame_id = frame_id

        fit_text = Path(fit_path).read_text(encoding="utf-8")

        rotation_degrees = read_fit_value(
            fit_text,
            "rotation_deg",
        )
        self.translation_x = read_fit_value(
            fit_text,
            "translation_x_m",
        )
        self.translation_y = read_fit_value(
            fit_text,
            "translation_y_m",
        )

        self.rotation_radians = math.radians(rotation_degrees)
        self.cosine = math.cos(self.rotation_radians)
        self.sine = math.sin(self.rotation_radians)

        with Path(csv_path).open(
            newline="",
            encoding="utf-8",
        ) as stream:
            self.rows = list(csv.DictReader(stream))

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(
            MarkerArray,
            topic,
            qos,
        )

        self.get_logger().info(
            f"Loaded {len(self.rows)} alignment pairs"
        )
        self.get_logger().info(
            f"Rotation: {rotation_degrees:+.9f} degrees"
        )
        self.get_logger().info(
            f"Translation: "
            f"({self.translation_x:+.6f}, "
            f"{self.translation_y:+.6f}) meters"
        )

        self.timer = self.create_timer(1.0, self.publish_preview)
        self.publish_preview()

    def transform(self, x, y):
        corrected_x = (
            self.cosine * x
            - self.sine * y
            + self.translation_x
        )
        corrected_y = (
            self.sine * x
            + self.cosine * y
            + self.translation_y
        )

        return corrected_x, corrected_y

    def publish_preview(self):
        stamp = self.get_clock().now().to_msg()
        message = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        message.markers.append(delete_all)

        for index, row in enumerate(self.rows):
            current_x = float(row["current_x"])
            current_y = float(row["current_y"])
            target_x = float(row["target_x"])
            target_y = float(row["target_y"])

            corrected_x, corrected_y = self.transform(
                current_x,
                current_y,
            )

            base_id = index * 10

            # Original QGIS/GPS-derived position: magenta.
            message.markers.append(
                make_sphere(
                    self.frame_id,
                    stamp,
                    "original_anchor",
                    base_id,
                    current_x,
                    current_y,
                    0.5,
                    0.65,
                    1.0,
                    0.0,
                    1.0,
                )
            )

            # Corrected transform preview: cyan.
            message.markers.append(
                make_sphere(
                    self.frame_id,
                    stamp,
                    "corrected_anchor",
                    base_id + 1,
                    corrected_x,
                    corrected_y,
                    0.7,
                    0.8,
                    0.0,
                    1.0,
                    1.0,
                )
            )

            # Manually clicked MOLA target: green.
            message.markers.append(
                make_sphere(
                    self.frame_id,
                    stamp,
                    "clicked_target",
                    base_id + 2,
                    target_x,
                    target_y,
                    0.5,
                    0.65,
                    0.0,
                    1.0,
                    0.0,
                )
            )

            # Residual line from corrected point to clicked target.
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = stamp
            line.ns = "remaining_error"
            line.id = base_id + 3
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.12

            line.color.r = 1.0
            line.color.g = 1.0
            line.color.b = 0.0
            line.color.a = 1.0

            corrected_point = Point()
            corrected_point.x = corrected_x
            corrected_point.y = corrected_y
            corrected_point.z = 0.7

            target_point = Point()
            target_point.x = target_x
            target_point.y = target_y
            target_point.z = 0.7

            line.points = [corrected_point, target_point]
            message.markers.append(line)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "alignment_preview_labels"
            label.id = base_id + 4
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose.position.x = corrected_x
            label.pose.position.y = corrected_y
            label.pose.position.z = 1.8
            label.pose.orientation.w = 1.0

            label.scale.z = 0.8
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0

            label.text = row["point_id"]
            message.markers.append(label)

        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pairs", required=True)
    parser.add_argument("--fit", required=True)
    parser.add_argument(
        "--topic",
        default="/alignment_transform_preview",
    )
    parser.add_argument(
        "--frame-id",
        default="map",
    )

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = PreviewPublisher(
        csv_path=args.pairs,
        fit_path=args.fit,
        topic=args.topic,
        frame_id=args.frame_id,
    )

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
