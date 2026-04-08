#!/usr/bin/env python3
"""Convert radar point clouds into clustered obstacles."""

import math
import struct

import rclpy
import tf2_ros
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from navigation_interface.msg import Obstacle, ObstacleArray
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray


class RadarObjectToObstacle(Node):
    def __init__(self):
        super().__init__("radar_object_to_obstacle")

        self.declare_parameter("points_topic", "/radar/points")
        self.declare_parameter("obstacles_topic", "/obstacles")
        self.declare_parameter("markers_topic", "/radar_obstacle_display")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("max_range_m", 12.0)
        self.declare_parameter("min_x_m", 0.5)
        self.declare_parameter("max_abs_y_m", 3.0)
        self.declare_parameter("max_abs_z_m", 2.0)
        self.declare_parameter("min_snr", 0.0)
        self.declare_parameter("cluster_tolerance_m", 0.75)
        self.declare_parameter("min_cluster_size", 3)
        self.declare_parameter("max_cluster_size", 40)
        self.declare_parameter("default_radius_m", 0.35)
        self.declare_parameter("max_radius_m", 1.5)

        self.points_topic = self.get_parameter("points_topic").value
        self.obstacles_topic = self.get_parameter("obstacles_topic").value
        self.markers_topic = self.get_parameter("markers_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.min_x_m = float(self.get_parameter("min_x_m").value)
        self.max_abs_y_m = float(self.get_parameter("max_abs_y_m").value)
        self.max_abs_z_m = float(self.get_parameter("max_abs_z_m").value)
        self.min_snr = float(self.get_parameter("min_snr").value)
        self.cluster_tolerance_m = float(
            self.get_parameter("cluster_tolerance_m").value
        )
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self.max_cluster_size = int(self.get_parameter("max_cluster_size").value)
        self.default_radius_m = float(self.get_parameter("default_radius_m").value)
        self.max_radius_m = float(self.get_parameter("max_radius_m").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.points_sub = self.create_subscription(
            PointCloud2, self.points_topic, self.points_callback, 10
        )
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, self.obstacles_topic, 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)

    def points_callback(self, msg: PointCloud2):
        points = self.decode_points(msg)
        filtered_points = [point for point in points if self.keep_point(point)]
        clusters = self.cluster_points(filtered_points)
        obstacles = self.build_obstacles(clusters, msg)
        self.obstacle_pub.publish(obstacles)
        self.marker_pub.publish(self.build_markers(obstacles))

    def decode_points(self, msg: PointCloud2):
        if msg.point_step < 24:
            self.get_logger().warn(
                f"Radar point_step {msg.point_step} is smaller than expected"
            )
            return []

        point_count = min(msg.width * msg.height, len(msg.data) // msg.point_step)
        points = []
        for index in range(point_count):
            offset = index * msg.point_step
            x, y, z, velocity, snr, noise = struct.unpack_from(
                "<ffffff", msg.data, offset
            )
            points.append(
                {
                    "x": x,
                    "y": y,
                    "z": z,
                    "velocity": velocity,
                    "snr": snr,
                    "noise": noise,
                    "frame_id": msg.header.frame_id,
                    "stamp": msg.header.stamp,
                }
            )
        return points

    def keep_point(self, point):
        range_xy = math.hypot(point["x"], point["y"])
        if range_xy > self.max_range_m:
            return False
        if point["x"] < self.min_x_m:
            return False
        if abs(point["y"]) > self.max_abs_y_m:
            return False
        if abs(point["z"]) > self.max_abs_z_m:
            return False
        if point["snr"] < self.min_snr:
            return False
        return True

    def cluster_points(self, points):
        clusters = []
        visited = [False] * len(points)

        for start_index in range(len(points)):
            if visited[start_index]:
                continue

            queue = [start_index]
            visited[start_index] = True
            cluster_indices = []

            while queue:
                current_index = queue.pop()
                cluster_indices.append(current_index)
                current_point = points[current_index]

                for neighbor_index in range(len(points)):
                    if visited[neighbor_index]:
                        continue
                    neighbor_point = points[neighbor_index]
                    if self.point_distance(current_point, neighbor_point) <= self.cluster_tolerance_m:
                        visited[neighbor_index] = True
                        queue.append(neighbor_index)

            if self.min_cluster_size <= len(cluster_indices) <= self.max_cluster_size:
                clusters.append([points[index] for index in cluster_indices])

        return clusters

    def point_distance(self, first_point, second_point):
        dx = first_point["x"] - second_point["x"]
        dy = first_point["y"] - second_point["y"]
        dz = first_point["z"] - second_point["z"]
        return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))

    def build_obstacles(self, clusters, msg: PointCloud2):
        obstacle_array = ObstacleArray()
        obstacle_array.header.frame_id = self.target_frame
        obstacle_array.header.stamp = msg.header.stamp

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, msg.header.stamp
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"Radar obstacle transform unavailable: {exc}")
            return obstacle_array

        for cluster in clusters:
            center_x = sum(point["x"] for point in cluster) / len(cluster)
            center_y = sum(point["y"] for point in cluster) / len(cluster)
            center_z = sum(point["z"] for point in cluster) / len(cluster)

            max_extent = max(
                math.sqrt(
                    ((point["x"] - center_x) ** 2)
                    + ((point["y"] - center_y) ** 2)
                    + ((point["z"] - center_z) ** 2)
                )
                for point in cluster
            )
            radius = min(self.max_radius_m, max(self.default_radius_m, max_extent))

            radar_point = PointStamped()
            radar_point.header.frame_id = msg.header.frame_id
            radar_point.header.stamp = msg.header.stamp
            radar_point.point.x = center_x
            radar_point.point.y = center_y
            radar_point.point.z = center_z

            transformed_point = do_transform_point(radar_point, transform)

            obstacle = Obstacle()
            obstacle.header.frame_id = self.target_frame
            obstacle.header.stamp = msg.header.stamp
            obstacle.pos = transformed_point
            obstacle.radius = radius
            obstacle.followable = True
            obstacle_array.obstacles.append(obstacle)

        return obstacle_array

    def build_markers(self, obstacles: ObstacleArray):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = self.target_frame
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "radar_obstacles"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        lifetime = Duration()
        lifetime.nanosec = int(2e8)

        for index, obstacle in enumerate(obstacles.obstacles):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = obstacles.header.stamp
            marker.ns = "radar_obstacles"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle.pos.point.x
            marker.pose.position.y = obstacle.pos.point.y
            marker.pose.position.z = obstacle.pos.point.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = obstacle.radius * 2.0
            marker.scale.y = obstacle.radius * 2.0
            marker.scale.z = 1.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.7
            marker.lifetime = lifetime
            marker_array.markers.append(marker)

        return marker_array


def main():
    rclpy.init()
    node = RadarObjectToObstacle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
