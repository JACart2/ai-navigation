#!/usr/bin/env python3

import argparse
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray


# Import the exact conversion utility used by navigation.
navigation_python_dir = Path(__file__).resolve().parents[1] / "navigation"
sys.path.insert(0, str(navigation_python_dir))

import simple_gps_util  # noqa: E402


def feature_is_enabled(value):
    if value is None or value == "":
        return True

    return str(value).strip().lower() not in {
        "0",
        "false",
        "no",
        "off",
        "disabled",
    }


class AlignmentPointPublisher(Node):
    def __init__(self, geojson_path, calibration_path, topic, frame_id):
        super().__init__("alignment_control_points_publisher")

        self.geojson_path = Path(geojson_path)
        self.calibration_path = Path(calibration_path)
        self.frame_id = frame_id
        self.last_mtime = None

        if not self.geojson_path.is_file():
            raise FileNotFoundError(
                f"Alignment GeoJSON not found: {self.geojson_path}"
            )

        if not self.calibration_path.is_file():
            raise FileNotFoundError(
                f"Calibration YAML not found: {self.calibration_path}"
            )

        local_points, gps_points = (
            simple_gps_util.load_landmark_calibration(
                str(self.calibration_path)
            )
        )

        self.calibration = simple_gps_util.calibrate_with_landmarks(
            local_points,
            gps_points,
        )

        (
            ref_lat,
            ref_lon,
            cx_local,
            cy_local,
            cx_gps,
            cy_gps,
            theta_degrees,
        ) = self.calibration

        self.get_logger().info(
            f"Loaded {len(local_points)} calibration landmarks"
        )
        self.get_logger().info(
            f"Calibration rotation: {theta_degrees:.6f} degrees"
        )
        self.get_logger().info(
            f"Reference GPS: {ref_lat:.9f}, {ref_lon:.9f}"
        )
        self.get_logger().info(
            f"Local centroid: ({cx_local:.3f}, {cy_local:.3f})"
        )
        self.get_logger().info(
            f"GPS centroid meters: ({cx_gps:.3f}, {cy_gps:.3f})"
        )

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

        self.timer = self.create_timer(1.0, self.publish_markers)
        self.publish_markers()

    def publish_markers(self):
        try:
            current_mtime = self.geojson_path.stat().st_mtime
            changed = current_mtime != self.last_mtime

            data = json.loads(
                self.geojson_path.read_text(encoding="utf-8")
            )
        except Exception as exc:
            self.get_logger().error(
                f"Could not read alignment points: {exc}"
            )
            return

        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        # Remove markers for points deleted from the file.
        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = now
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        published_count = 0

        for feature_index, feature in enumerate(
            data.get("features", [])
        ):
            properties = feature.get("properties") or {}

            if not feature_is_enabled(properties.get("enabled")):
                continue

            geometry = feature.get("geometry") or {}
            coordinates = geometry.get("coordinates") or []

            if geometry.get("type") != "Point" or len(coordinates) < 2:
                self.get_logger().warning(
                    f"Skipping feature {feature_index}: "
                    "not a valid Point geometry"
                )
                continue

            longitude = float(coordinates[0])
            latitude = float(coordinates[1])

            map_x, map_y = simple_gps_util.gps_to_local(
                latitude,
                longitude,
                *self.calibration,
            )

            label = (
                properties.get("label")
                or properties.get("point_id")
                or properties.get("building")
                or f"anchor_{feature_index}"
            )

            marker_id = published_count * 2

            point_marker = Marker()
            point_marker.header.frame_id = self.frame_id
            point_marker.header.stamp = now
            point_marker.ns = "alignment_anchor"
            point_marker.id = marker_id
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD

            point_marker.pose.position.x = float(map_x)
            point_marker.pose.position.y = float(map_y)
            point_marker.pose.position.z = 0.75
            point_marker.pose.orientation.w = 1.0

            point_marker.scale.x = 1.5
            point_marker.scale.y = 1.5
            point_marker.scale.z = 1.5

            point_marker.color.r = 1.0
            point_marker.color.g = 0.0
            point_marker.color.b = 1.0
            point_marker.color.a = 1.0

            marker_array.markers.append(point_marker)

            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = now
            text_marker.ns = "alignment_label"
            text_marker.id = marker_id + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = float(map_x)
            text_marker.pose.position.y = float(map_y)
            text_marker.pose.position.z = 2.2
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 1.2

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = str(label)

            marker_array.markers.append(text_marker)

            if changed:
                self.get_logger().info(
                    f"{label}: "
                    f"GPS=({latitude:.9f}, {longitude:.9f}) "
                    f"map=({map_x:.3f}, {map_y:.3f})"
                )

            published_count += 1

        self.publisher.publish(marker_array)

        if changed:
            self.last_mtime = current_mtime
            self.get_logger().info(
                f"Published {published_count} alignment points"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--geojson", required=True)
    parser.add_argument("--calibration", required=True)
    parser.add_argument(
        "--topic",
        default="/alignment_control_points",
    )
    parser.add_argument(
        "--frame-id",
        default="map",
    )

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    try:
        node = AlignmentPointPublisher(
            geojson_path=args.geojson,
            calibration_path=args.calibration,
            topic=args.topic,
            frame_id=args.frame_id,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
