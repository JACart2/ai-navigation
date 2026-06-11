#!/usr/bin/env python3

import numpy as np
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
    """Republish Velodyne point clouds in a stable frame with TF fallbacks."""

    def __init__(self):
        super().__init__("velodyne_pointcloud_tf_fallback")

        # PointCloud2 topic produced by the Velodyne pointcloud converter.
        self.declare_parameter("input_topic", "/velodyne_points")
        # PointCloud2 topic this node publishes after optional downsampling/TF.
        self.declare_parameter("output_topic", "/velodyne_points_stable")
        # Final frame the cloud should be published in, usually "map".
        self.declare_parameter("target_frame", "map")
        # Vehicle/body frame that the lidar is mounted to.
        self.declare_parameter("parent_frame", "base_link")
        # Lidar sensor frame expected on incoming Velodyne clouds.
        self.declare_parameter("child_frame", "velodyne")
        # Hardcoded lidar X offset from parent_frame, in meters.
        self.declare_parameter("x", 1.0)
        # Hardcoded lidar Y offset from parent_frame, in meters.
        self.declare_parameter("y", 0.0)
        # Hardcoded lidar Z offset from parent_frame, in meters.
        self.declare_parameter("z", 1.9)
        # Hardcoded lidar roll from parent_frame, in radians.
        self.declare_parameter("roll", 0.0)
        # Hardcoded lidar pitch from parent_frame, in radians.
        self.declare_parameter("pitch", 0.0)
        # Hardcoded lidar yaw from parent_frame, in radians.
        self.declare_parameter("yaw", 0.0)
        # Max time to wait for each TF lookup before using fallback behavior.
        self.declare_parameter("lookup_timeout_s", 0.05)
        # How often to refresh the latest base_link -> target_frame transform.
        self.declare_parameter("cart_pose_refresh_period_s", 0.1)
        # Force the hardcoded lidar transform even if TF has a live lidar transform.
        self.declare_parameter("prefer_fallback_lidar_tf", False)
        # If the incoming cloud is already in target_frame, publish it directly.
        self.declare_parameter("republish_original_if_possible", True)
        # Keep one point every N columns in each cloud row. 1 disables downsampling.
        self.declare_parameter("downsample_stride", 1)

        # Cache parameters that are used on every cloud callback.
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
        # Static lidar mounting transform used when the live lidar TF is missing.
        self.fallback_transform = self._build_fallback_transform()
        # Last good cart pose transform, reused during short TF dropouts.
        self.last_base_to_target_transform = None
        # These flags keep fallback/restored log messages from repeating every scan.
        self.using_fallback = False
        self.using_cached_target_transform = False

        # TF buffer/listener receive transforms published by the rest of the ROS graph.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Periodically cache the latest cart pose so pointcloud callbacks stay fast.
        self.cart_pose_refresh_timer = self.create_timer(
            float(self.get_parameter("cart_pose_refresh_period_s").value),
            self._refresh_latest_base_to_target_transform,
        )

        # Sensor-data QoS matches high-rate point cloud streams better than default QoS.
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
        """Downsample and transform one incoming PointCloud2 message."""
        cloud = self._downsample_cloud(cloud)

        # Avoid unnecessary TF work when the incoming cloud is already stable.
        if self._can_republish_original(cloud.header.frame_id):
            self.publisher.publish(cloud)
            return

        # First find lidar/source_frame -> base_link. This may use the hardcoded mount.
        lidar_to_base = self._lookup_lidar_to_base_transform(
            cloud.header.frame_id, cloud.header.stamp
        )
        if lidar_to_base is None:
            return

        # Then find base_link -> target_frame. If target_frame is base_link, this is None.
        base_to_target = self._lookup_base_to_target_transform(cloud.header.stamp)
        if base_to_target is not None:
            # Compose both transforms so the cloud can be transformed in one operation.
            output_transform = self._compose_transforms(
                base_to_target, lidar_to_base, cloud.header.stamp
            )
        else:
            output_transform = lidar_to_base

        # Apply the selected transform and publish the cloud in the output frame.
        transformed_cloud = do_transform_cloud(cloud, output_transform)
        transformed_cloud.header.stamp = cloud.header.stamp
        transformed_cloud.header.frame_id = output_transform.header.frame_id

        self.publisher.publish(transformed_cloud)

    def _can_republish_original(self, source_frame: str):
        """Return True when the cloud is already in the requested output frame."""
        return self.republish_original_if_possible and self._normalize_frame(
            source_frame
        ) == self._normalize_frame(self.target_frame)

    def _downsample_cloud(self, cloud: PointCloud2) -> PointCloud2:
        """Drop point columns by stride while preserving PointCloud2 metadata."""
        if self.downsample_stride <= 1 or cloud.width == 0 or cloud.point_step == 0:
            return cloud

        # Width is the number of points per row; stride keeps columns 0, N, 2N, ...
        row_width = cloud.width
        new_width = (row_width + self.downsample_stride - 1) // self.downsample_stride
        if new_width == row_width:
            return cloud

        point_step = cloud.point_step
        row_step = cloud.row_step if cloud.row_step else row_width * point_step
        downsampled = bytearray()

        # Copy each selected point as raw bytes so field layout stays untouched.
        for row in range(max(1, cloud.height)):
            row_start = row * row_step
            for col in range(0, row_width, self.downsample_stride):
                point_start = row_start + col * point_step
                downsampled.extend(cloud.data[point_start : point_start + point_step])

        # Rebuild the message around the smaller byte buffer.
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
        """Find source/lidar -> base_link, using the hardcoded mount if needed."""
        if self.prefer_fallback_lidar_tf and self._normalize_frame(
            source_frame
        ) == self._normalize_frame(self.child_frame):
            # Configured override: always trust the launch-file lidar mount values.
            if not self.using_fallback:
                self.get_logger().info(
                    f"PointCloudVelodyne using preferred hardcoded transform "
                    f"for {source_frame} -> {self.parent_frame}"
                )
                self.using_fallback = True
            self.fallback_transform.header.stamp = stamp
            return self.fallback_transform

        try:
            # Normal path: use the live TF tree at the pointcloud timestamp.
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
            # Only the configured lidar frame can safely use the hardcoded fallback.
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

        # Live lidar TF is missing, so use the static mount configured by parameters.
        self.fallback_transform.header.stamp = stamp
        return self.fallback_transform

    def _lookup_base_to_target_transform(self, stamp):
        """Find base_link -> target_frame, using the cached cart pose during dropouts."""
        if self._normalize_frame(self.target_frame) == self._normalize_frame(
            self.parent_frame
        ):
            return None

        if self.last_base_to_target_transform is not None:
            # Use the cached cart pose immediately; the timer keeps it fresh.
            self.last_base_to_target_transform.header.stamp = stamp
            return self.last_base_to_target_transform

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
        """Timer callback that refreshes the cached cart pose transform."""
        self._lookup_latest_base_to_target_transform()

    def _lookup_latest_base_to_target_transform(self):
        """Look up the newest available base_link -> target_frame transform."""
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
        """Build the hardcoded lidar mount transform from launch parameters."""
        transform = TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = float(self.get_parameter("x").value)
        transform.transform.translation.y = float(self.get_parameter("y").value)
        transform.transform.translation.z = float(self.get_parameter("z").value)

        # TF stores rotation as a quaternion, so convert launch-file roll/pitch/yaw.
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

    def _compose_transforms(
        self,
        target_to_parent: TransformStamped,
        parent_to_source: TransformStamped,
        stamp,
    ) -> TransformStamped:
        """Compose target->parent and parent->source into target->source."""
        target_to_parent_rotation = self._quaternion_from_transform(target_to_parent)
        parent_to_source_translation = self._translation_from_transform(
            parent_to_source
        )
        # Rotate the source offset into the target frame before adding translations.
        rotated_translation = tf_transformations.quaternion_matrix(
            target_to_parent_rotation
        )[:3, :3].dot(parent_to_source_translation)

        composed_translation = (
            self._translation_from_transform(target_to_parent) + rotated_translation
        )
        composed_rotation = tf_transformations.quaternion_multiply(
            target_to_parent_rotation,
            self._quaternion_from_transform(parent_to_source),
        )

        # Package the composed translation and rotation back into a ROS transform.
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = target_to_parent.header.frame_id
        transform.child_frame_id = parent_to_source.child_frame_id
        transform.transform.translation.x = float(composed_translation[0])
        transform.transform.translation.y = float(composed_translation[1])
        transform.transform.translation.z = float(composed_translation[2])
        transform.transform.rotation.x = float(composed_rotation[0])
        transform.transform.rotation.y = float(composed_rotation[1])
        transform.transform.rotation.z = float(composed_rotation[2])
        transform.transform.rotation.w = float(composed_rotation[3])
        return transform

    @staticmethod
    def _translation_from_transform(transform: TransformStamped):
        """Return transform translation as a numpy vector."""
        translation = transform.transform.translation
        return np.array([translation.x, translation.y, translation.z])

    @staticmethod
    def _quaternion_from_transform(transform: TransformStamped):
        """Return transform rotation as [x, y, z, w]."""
        rotation = transform.transform.rotation
        return [rotation.x, rotation.y, rotation.z, rotation.w]

    @staticmethod
    def _normalize_frame(frame_id):
        """Treat '/frame' and 'frame' as the same ROS frame name."""
        return frame_id.lstrip("/") if frame_id else frame_id


def main():
    """Start the ROS node and spin until shutdown."""
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
