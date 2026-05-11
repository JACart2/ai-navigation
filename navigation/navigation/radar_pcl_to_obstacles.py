#!/usr/bin/env python3
"""Convert filtered radar PointCloud2 directly into obstacles.

The existing radar->LaserScan->cluster pipeline in lidar_object_to_obstacle
loses sparse radar returns: the cluster step requires consecutive finite-range
angle bins within 0.10m, which sparse radar rarely produces. This node skips
LaserScan entirely — it groups the filtered radar points by 3D distance,
transforms each cluster to base_link, and publishes them as obstacles.

Subscribes:
    /ti_mmwave/radar_scan_pcl_filtered   (sensor_msgs/PointCloud2)
Publishes:
    /obstacles                           (navigation_interface/ObstacleArray)
    /radar_obstacle_display              (visualization_msgs/Marker)
"""
from __future__ import annotations

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  Needed to register PointStamped transforms.
import tf2_ros
from geometry_msgs.msg import PointStamped
from navigation_interface.msg import Obstacle, ObstacleArray
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker


class RadarPCLToObstacles(Node):
    def __init__(self):
        super().__init__("radar_pcl_to_obstacles")

        self.declare_parameter("input_topic", "/ti_mmwave/radar_scan_pcl_filtered")
        self.declare_parameter("obstacles_topic", "/obstacles")
        self.declare_parameter("marker_topic", "/radar_obstacle_display")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("cluster_radius", 0.6)
        self.declare_parameter("min_cluster_points", 1)
        self.declare_parameter("min_obstacle_radius", 0.30)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.cluster_radius = float(self.get_parameter("cluster_radius").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.min_obstacle_radius = float(self.get_parameter("min_obstacle_radius").value)

        in_topic = str(self.get_parameter("input_topic").value)
        obstacles_topic = str(self.get_parameter("obstacles_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(PointCloud2, in_topic, self._on_cloud, 10)
        self.obstacles_pub = self.create_publisher(ObstacleArray, obstacles_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)

        self.get_logger().info(
            f"radar_pcl_to_obstacles: {in_topic} -> {obstacles_topic} "
            f"(target_frame={self.target_frame}, cluster_r={self.cluster_radius}, "
            f"min_pts={self.min_cluster_points})"
        )

    def _on_cloud(self, msg: PointCloud2):
        xyz = self._extract_xyz(msg)
        out = ObstacleArray()
        out.header.frame_id = self.target_frame
        out.header.stamp = msg.header.stamp

        if xyz.shape[0] == 0:
            self.obstacles_pub.publish(out)
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF {msg.header.frame_id}->{self.target_frame} failed: {e}",
                throttle_duration_sec=2.0,
            )
            return

        clusters = self._cluster(xyz)

        for i, cluster in enumerate(clusters):
            if cluster.shape[0] < self.min_cluster_points:
                continue

            centroid = np.mean(cluster, axis=0)
            spread = (
                float(np.max(np.linalg.norm(cluster - centroid, axis=1)))
                if cluster.shape[0] > 1
                else 0.0
            )
            radius = max(self.min_obstacle_radius, spread)

            ps = PointStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.header.stamp = msg.header.stamp
            ps.point.x = float(centroid[0])
            ps.point.y = float(centroid[1])
            ps.point.z = float(centroid[2])

            try:
                ps_out = tf2_geometry_msgs.do_transform_point(ps, tf)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(
                    f"Transform error: {e}", throttle_duration_sec=2.0
                )
                continue

            obs = Obstacle()
            obs.header.frame_id = self.target_frame
            obs.header.stamp = msg.header.stamp
            obs.pos = ps_out
            obs.radius = float(radius)
            obs.followable = True
            out.obstacles.append(obs)

            self.marker_pub.publish(self._marker(ps_out, radius, i, msg.header.stamp))

        self.obstacles_pub.publish(out)

    def _extract_xyz(self, msg: PointCloud2) -> np.ndarray:
        fields = {f.name: f for f in msg.fields}
        if not all(n in fields for n in ("x", "y", "z")):
            return np.empty((0, 3), dtype=np.float32)
        for n in ("x", "y", "z"):
            if fields[n].datatype != PointField.FLOAT32:
                return np.empty((0, 3), dtype=np.float32)
        n_pts = msg.width * msg.height
        if n_pts == 0 or msg.point_step == 0:
            return np.empty((0, 3), dtype=np.float32)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size != n_pts * msg.point_step:
            return np.empty((0, 3), dtype=np.float32)
        raw = raw.reshape(n_pts, msg.point_step)
        endian = ">" if msg.is_bigendian else "<"
        out = np.zeros((n_pts, 3), dtype=np.float32)
        for j, name in enumerate(("x", "y", "z")):
            off = fields[name].offset
            out[:, j] = np.frombuffer(
                raw[:, off:off + 4].tobytes(), dtype=f"{endian}f4"
            )
        return out

    def _cluster(self, xyz: np.ndarray):
        """Greedy 3D distance clustering — first cluster within radius wins."""
        clusters = []  # list of {points: list[np.array], centroid: np.array}
        thr = self.cluster_radius
        for p in xyz:
            assigned = False
            for c in clusters:
                if np.linalg.norm(c["centroid"] - p) < thr:
                    c["points"].append(p)
                    c["centroid"] = np.mean(c["points"], axis=0)
                    assigned = True
                    break
            if not assigned:
                clusters.append({"points": [p], "centroid": p.copy()})
        return [np.asarray(c["points"]) for c in clusters]

    def _marker(self, ps: PointStamped, radius: float, idx: int, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = stamp
        m.ns = "Radar_Objects"
        m.id = idx
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = ps.point.x
        m.pose.position.y = ps.point.y
        m.pose.position.z = ps.point.z
        m.pose.orientation.w = 1.0
        diameter = max(0.4, radius * 2.0)
        m.scale.x = diameter
        m.scale.y = diameter
        m.scale.z = 0.2
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 0.85
        m.lifetime = Duration(seconds=0.3).to_msg()
        return m


def main():
    rclpy.init()
    node = RadarPCLToObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
