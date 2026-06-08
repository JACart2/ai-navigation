#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class TfFallbackPublisher(Node):
    def __init__(self):
        super().__init__("tf_fallback_publisher")

        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "velodyne")
        self.declare_parameter("x", 1.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 1.9)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("check_period_s", 0.2)
        self.declare_parameter("missing_checks_before_fallback", 3)
        self.declare_parameter("lookup_timeout_s", 0.05)

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.missing_checks_before_fallback = int(
            self.get_parameter("missing_checks_before_fallback").value
        )
        self.lookup_timeout = Duration(
            seconds=float(self.get_parameter("lookup_timeout_s").value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.fallback_transform = self._build_transform()
        self.missing_checks = 0
        self.fallback_active = False

        self.timer = self.create_timer(
            float(self.get_parameter("check_period_s").value), self._check_tf
        )

    def _build_transform(self) -> TransformStamped:
        transform = TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = float(self.get_parameter("x").value)
        transform.transform.translation.y = float(self.get_parameter("y").value)
        transform.transform.translation.z = float(self.get_parameter("z").value)

        roll = float(self.get_parameter("roll").value)
        pitch = float(self.get_parameter("pitch").value)
        yaw = float(self.get_parameter("yaw").value)
        qx, qy, qz, qw = self._quaternion_from_euler(roll, pitch, yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    def _check_tf(self):
        if self.fallback_active:
            self.fallback_transform.header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(self.fallback_transform)
            return

        if self._normal_transform_available():
            self.missing_checks = 0
            return

        self.missing_checks += 1
        if self.missing_checks < self.missing_checks_before_fallback:
            return

        self.fallback_transform.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.fallback_transform)

        if not self.fallback_active:
            self.fallback_active = True
            self.get_logger().warn(
                f"Normal TF {self.parent_frame} -> {self.child_frame} is unavailable; "
                "publishing fallback transform"
            )

    def _normal_transform_available(self) -> bool:
        try:
            return self.tf_buffer.can_transform(
                self.parent_frame,
                self.child_frame,
                Time(),
                timeout=self.lookup_timeout,
            )
        except TransformException:
            return False

    @staticmethod
    def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main():
    rclpy.init()
    node = TfFallbackPublisher()
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
