#!/usr/bin/env python3
"""Crop a TI mmWave radar PointCloud2 to a configurable x/y/z box.

Subscribes:
    /ti_mmwave/radar_scan_pcl   (sensor_msgs/PointCloud2)

Publishes:
    /ti_mmwave/radar_scan_pcl_filtered (sensor_msgs/PointCloud2)
    /radar_filter_box                  (visualization_msgs/Marker)

Parameters (all dynamic; change them live with ros2 param set or the slider GUI):
    x_min, x_max, y_min, y_max, z_min, z_max  (double, meters)
    input_topic, output_topic, marker_topic    (string)
    marker_frame                               (string, frame to render box in;
                                                defaults to the incoming cloud's frame)

Filtering happens in the radar's own frame (ti_mmwave_0). Coordinate convention:
    x forward, y left, z up. The radar is mounted ~0.9 m above the ground, so
    ground points sit near z = -0.9.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rcl_interfaces.msg import (
    FloatingPointRange,
    ParameterDescriptor,
    SetParametersResult,
)
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


PARAM_BOUNDS = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")

DEFAULT_BOUNDS = {
    "x_min": 0.25,
    "x_max": 3.58,
    "y_min": -2.10,
    "y_max": 5.00,
    "z_min": -0.65,
    "z_max": 1.11,
}

SLIDER_LIMITS = {
    "x_min": (-10.0, 10.0),
    "x_max": (-10.0, 10.0),
    "y_min": (-10.0, 10.0),
    "y_max": (-10.0, 10.0),
    "z_min": (-3.0, 3.0),
    "z_max": (-3.0, 3.0),
}


def _bounded_descriptor(name: str) -> ParameterDescriptor:
    lo, hi = SLIDER_LIMITS[name]
    desc = ParameterDescriptor()
    desc.description = f"Radar filter {name} (meters, radar frame)"
    desc.floating_point_range = [FloatingPointRange(from_value=lo, to_value=hi, step=0.0)]
    return desc


class RadarXYZFilter(Node):
    def __init__(self):
        super().__init__("radar_xyz_filter")

        self.declare_parameter("input_topic", "/ti_mmwave/radar_scan_pcl")
        self.declare_parameter("output_topic", "/ti_mmwave/radar_scan_pcl_filtered")
        self.declare_parameter("rejected_topic", "/ti_mmwave/radar_scan_pcl_rejected")
        self.declare_parameter("marker_topic", "/radar_filter_box")
        self.declare_parameter("marker_frame", "")

        for name in PARAM_BOUNDS:
            self.declare_parameter(name, DEFAULT_BOUNDS[name], _bounded_descriptor(name))

        self.bounds = {n: self.get_parameter(n).value for n in PARAM_BOUNDS}

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        rejected_topic = self.get_parameter("rejected_topic").value
        marker_topic = self.get_parameter("marker_topic").value

        self.sub = self.create_subscription(PointCloud2, in_topic, self._on_cloud, 10)
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.rejected_pub = self.create_publisher(PointCloud2, rejected_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)

        self.add_on_set_parameters_callback(self._on_params)
        self.marker_timer = self.create_timer(0.2, self._publish_marker)
        self._last_frame_id = ""

        self.get_logger().info(
            f"radar_xyz_filter ready: {in_topic} -> {out_topic}  bounds={self.bounds}"
        )

    def _on_params(self, params):
        for p in params:
            if p.name in PARAM_BOUNDS:
                try:
                    self.bounds[p.name] = float(p.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False,
                                               reason=f"{p.name} must be a number")
        if self.bounds["x_min"] > self.bounds["x_max"] \
                or self.bounds["y_min"] > self.bounds["y_max"] \
                or self.bounds["z_min"] > self.bounds["z_max"]:
            self.get_logger().warn(
                "Filter bounds have min > max on some axis — output will be empty."
            )
        return SetParametersResult(successful=True)

    def _on_cloud(self, msg: PointCloud2):
        self._last_frame_id = msg.header.frame_id
        kept, rejected = self._split(msg)
        self.pub.publish(kept)
        self.rejected_pub.publish(rejected)

    def _empty_like(self, msg: PointCloud2) -> PointCloud2:
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = 0
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = 0
        out.is_dense = True
        out.data = b""
        return out

    def _subset(self, msg: PointCloud2, raw: np.ndarray, mask: np.ndarray) -> PointCloud2:
        kept = raw[mask]
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(kept.shape[0])
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * out.width
        out.is_dense = True
        out.data = kept.tobytes()
        return out

    def _split(self, msg: PointCloud2):
        """Return (kept, rejected) PointCloud2 messages."""
        fields_by_name = {f.name: f for f in msg.fields}
        if not all(n in fields_by_name for n in ("x", "y", "z")):
            self.get_logger().warn_once(
                "Incoming cloud is missing x/y/z fields; passing through unfiltered."
            )
            return msg, self._empty_like(msg)

        for n in ("x", "y", "z"):
            if fields_by_name[n].datatype != PointField.FLOAT32:
                self.get_logger().warn_once(
                    f"{n} field is not FLOAT32; passing through unfiltered."
                )
                return msg, self._empty_like(msg)

        n_points = msg.width * msg.height
        if n_points == 0 or msg.point_step == 0:
            return msg, self._empty_like(msg)

        endian = ">" if msg.is_bigendian else "<"
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size != n_points * msg.point_step:
            return msg, self._empty_like(msg)
        raw = raw.reshape(n_points, msg.point_step)

        def coord(name: str) -> np.ndarray:
            off = fields_by_name[name].offset
            return np.frombuffer(
                raw[:, off:off + 4].tobytes(), dtype=f"{endian}f4"
            )

        x = coord("x")
        y = coord("y")
        z = coord("z")

        b = self.bounds
        inside = (
            (x >= b["x_min"]) & (x <= b["x_max"]) &
            (y >= b["y_min"]) & (y <= b["y_max"]) &
            (z >= b["z_min"]) & (z <= b["z_max"])
        )
        return self._subset(msg, raw, inside), self._subset(msg, raw, ~inside)

    def _publish_marker(self):
        frame = self.get_parameter("marker_frame").value or self._last_frame_id
        if not frame:
            return
        b = self.bounds
        corners = [
            (b["x_min"], b["y_min"], b["z_min"]),
            (b["x_max"], b["y_min"], b["z_min"]),
            (b["x_max"], b["y_max"], b["z_min"]),
            (b["x_min"], b["y_max"], b["z_min"]),
            (b["x_min"], b["y_min"], b["z_max"]),
            (b["x_max"], b["y_min"], b["z_max"]),
            (b["x_max"], b["y_max"], b["z_max"]),
            (b["x_min"], b["y_max"], b["z_max"]),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "radar_filter_box"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 0.9
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.pose.orientation.w = 1.0
        for a, c in edges:
            for idx in (a, c):
                cx, cy, cz = corners[idx]
                marker.points.append(Point(x=float(cx), y=float(cy), z=float(cz)))
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = RadarXYZFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
