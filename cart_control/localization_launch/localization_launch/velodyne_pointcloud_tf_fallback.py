#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf_transformations


class VelodynePointcloudTfFallback(Node):
    def __init__(self):
        super().__init__("velodyne_pointcloud_tf_fallback")

        self.declare_parameter("input_topic", "/velodyne_points")
        self.declare_parameter("output_topic", "/velodyne_points_stable")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "velodyne")
        self.declare_parameter("x", 1.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 1.9)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("lookup_timeout_s", 0.05)
        self.declare_parameter("cart_pose_refresh_period_s", 0.1)
        self.declare_parameter("prefer_fallback_lidar_tf", False)
        self.declare_parameter("republish_original_if_possible", True)
        self.declare_parameter("downsample_stride", 1)

        self.target_frame = self.get_parameter("target_frame").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.lookup_timeout = Duration(
            seconds=float(self.get_parameter("lookup_timeout_s").value)
        )
        self.prefer_fallback_lidar_tf = bool(
            self.get_parameter("prefer_fallback_lidar_tf").value
        )
        self.republish_original_if_possible = bool(
            self.get_parameter("republish_original_if_possible").value
        )
        self.downsample_stride = max(
            1, int(self.get_parameter("downsample_stride").value)
        )
        self.fallback_transform = self._build_fallback_transform()
        self.last_base_to_target_transform = None
        self.using_fallback = False
        self.using_cached_target_transform = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cart_pose_refresh_timer = self.create_timer(
            float(self.get_parameter("cart_pose_refresh_period_s").value),
            self._refresh_latest_base_to_target_transform,
        )

        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self._pointcloud_callback,
            qos_profile_sensor_data,
        )

    def _pointcloud_callback(self, cloud: PointCloud2):
        if self._can_republish_original(cloud.header.frame_id):
            self.publisher.publish(self._downsample_cloud(cloud))
            return

        lidar_to_base = self._lookup_lidar_to_base_transform(
            cloud.header.frame_id, cloud.header.stamp
        )
        if lidar_to_base is None:
            return

        transformed_cloud = do_transform_cloud(cloud, lidar_to_base)
        transformed_cloud.header.stamp = cloud.header.stamp
        transformed_cloud.header.frame_id = self.parent_frame

        base_to_target = self._lookup_base_to_target_transform(cloud.header.stamp)
        if base_to_target is not None:
            transformed_cloud = do_transform_cloud(transformed_cloud, base_to_target)
            transformed_cloud.header.stamp = cloud.header.stamp
            transformed_cloud.header.frame_id = self.target_frame

        self.publisher.publish(self._downsample_cloud(transformed_cloud))

    def _can_republish_original(self, source_frame: str):
        return self.republish_original_if_possible and self._normalize_frame(
            source_frame
        ) == self._normalize_frame(self.target_frame)

    def _downsample_cloud(self, cloud: PointCloud2) -> PointCloud2:
        if self.downsample_stride <= 1 or cloud.width == 0 or cloud.point_step == 0:
            return cloud

        row_width = cloud.width
        new_width = (row_width + self.downsample_stride - 1) // self.downsample_stride
        if new_width == row_width:
            return cloud

        point_step = cloud.point_step
        row_step = cloud.row_step if cloud.row_step else row_width * point_step
        downsampled = bytearray()

        for row in range(max(1, cloud.height)):
            row_start = row * row_step
            for col in range(0, row_width, self.downsample_stride):
                point_start = row_start + col * point_step
                downsampled.extend(cloud.data[point_start : point_start + point_step])

        downsampled_cloud = PointCloud2()
        downsampled_cloud.header = cloud.header
        downsampled_cloud.height = cloud.height
        downsampled_cloud.width = new_width
        downsampled_cloud.fields = cloud.fields
        downsampled_cloud.is_bigendian = cloud.is_bigendian
        downsampled_cloud.point_step = point_step
        downsampled_cloud.row_step = new_width * point_step
        downsampled_cloud.data = bytes(downsampled)
        downsampled_cloud.is_dense = cloud.is_dense
        return downsampled_cloud

    def _lookup_lidar_to_base_transform(self, source_frame: str, stamp):
        if self.prefer_fallback_lidar_tf and self._normalize_frame(
            source_frame
        ) == self._normalize_frame(self.child_frame):
            if not self.using_fallback:
                self.get_logger().info(
                    f"PointCloudVelodyne using preferred hardcoded transform "
                    f"for {source_frame} -> {self.parent_frame}"
                )
                self.using_fallback = True
            self.fallback_transform.header.stamp = stamp
            return self.fallback_transform

        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=self.lookup_timeout,
            )
            if self.using_fallback:
                self.get_logger().info(
                    f"PointCloudVelodyne TF restored for {source_frame} -> {self.parent_frame}"
                )
                self.using_fallback = False
            return transform
        except TransformException as exc:
            if self._normalize_frame(source_frame) != self._normalize_frame(
                self.child_frame
            ):
                self.get_logger().warn(
                    f"PointCloudVelodyne could not transform {source_frame} -> "
                    f"{self.parent_frame}: {exc}"
                )
                return None

        if not self.using_fallback:
            self.get_logger().warn(
                f"PointCloudVelodyne TF unavailable for {source_frame} -> "
                f"{self.parent_frame}; using hardcoded Velodyne transform"
            )
            self.using_fallback = True

        self.fallback_transform.header.stamp = stamp
        return self.fallback_transform

    def _lookup_base_to_target_transform(self, stamp):
        if self._normalize_frame(self.target_frame) == self._normalize_frame(
            self.parent_frame
        ):
            return None

        latest_transform = self._lookup_latest_base_to_target_transform()
        if latest_transform is not None:
            return latest_transform

        # If the latest lookup is briefly unavailable, try the pointcloud stamp
        # before falling back to the cached cart pose.
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.parent_frame,
                Time.from_msg(stamp),
                timeout=self.lookup_timeout,
            )
            self.last_base_to_target_transform = transform
            if self.using_cached_target_transform:
                self.get_logger().info(
                    f"PointCloudVelodyne TF restored for {self.parent_frame} -> {self.target_frame}"
                )
                self.using_cached_target_transform = False
            return transform
        except TransformException as exc:
            if self.last_base_to_target_transform is None:
                self.get_logger().warn(
                    f"PointCloudVelodyne could not transform {self.parent_frame} -> "
                    f"{self.target_frame}: {exc}"
                )
                return None

        if not self.using_cached_target_transform:
            self.get_logger().warn(
                f"PointCloudVelodyne TF unavailable for {self.parent_frame} -> "
                f"{self.target_frame}; using last known transform"
            )
            self.using_cached_target_transform = True

        self.last_base_to_target_transform.header.stamp = stamp
        return self.last_base_to_target_transform

    def _refresh_latest_base_to_target_transform(self):
        self._lookup_latest_base_to_target_transform()

    def _lookup_latest_base_to_target_transform(self):
        if self._normalize_frame(self.target_frame) == self._normalize_frame(
            self.parent_frame
        ):
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.parent_frame,
                Time(),
                timeout=self.lookup_timeout,
            )
            self.last_base_to_target_transform = transform
            if self.using_cached_target_transform:
                self.get_logger().info(
                    f"PointCloudVelodyne TF restored for {self.parent_frame} -> {self.target_frame}"
                )
                self.using_cached_target_transform = False
            return transform
        except TransformException:
            return None

    def _build_fallback_transform(self):
        transform = TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = float(self.get_parameter("x").value)
        transform.transform.translation.y = float(self.get_parameter("y").value)
        transform.transform.translation.z = float(self.get_parameter("z").value)

        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
            float(self.get_parameter("roll").value),
            float(self.get_parameter("pitch").value),
            float(self.get_parameter("yaw").value),
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    @staticmethod
    def _normalize_frame(frame_id):
        return frame_id.lstrip("/") if frame_id else frame_id


def main():
    rclpy.init()
    node = VelodynePointcloudTfFallback()
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
