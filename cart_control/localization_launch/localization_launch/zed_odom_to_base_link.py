#!/usr/bin/env python3
"""
Converts ZED front-camera odometry (child_frame_id = camera tracking link) to
base_link odometry by composing with the static TF offset from camera to base.

Subscribes:  /zed_front/zed_node_0/odom  (nav_msgs/Odometry)
Publishes:   /odom                        (nav_msgs/Odometry, child_frame_id=base_link)

ZED TF publishing is disabled so we don't touch the TF tree; this node converts
the odom topic instead.
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
import tf2_ros


class ZedOdomToBaseLink(Node):
    def __init__(self):
        super().__init__('zed_odom_to_base_link')

        self.declare_parameter('zed_odom_topic', '/zed_front/zed_node_0/odom')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')

        zed_topic = self.get_parameter('zed_odom_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_odom_topic').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Cached T_{cam←base}: how base_link is expressed in camera frame.
        # t_cb: np.ndarray(3,), R_cb: Rotation — filled on first successful TF lookup.
        self._t_cb = None
        self._R_cb = None

        self._pub = self.create_publisher(Odometry, out_topic, qos_profile_sensor_data)
        self._sub = self.create_subscription(
            Odometry, zed_topic, self._callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'zed_odom_to_base_link: {zed_topic} -> {out_topic} (base_frame={self._base_frame})'
        )

    def _fetch_static_tf(self, camera_frame: str) -> bool:
        """
        Cache T_{cam←base}.
        lookup_transform(target=camera_frame, source=base_frame) gives the transform
        that maps base_link coordinates into camera frame, so:
          t = position of base_link origin in camera frame
          R = rotation of base_link in camera frame
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                camera_frame, self._base_frame, rclpy.time.Time()
            )
            tr = tf.transform.translation
            ro = tf.transform.rotation
            self._t_cb = np.array([tr.x, tr.y, tr.z])
            self._R_cb = Rotation.from_quat([ro.x, ro.y, ro.z, ro.w])
            self.get_logger().info(
                f'TF {camera_frame}<-{self._base_frame} cached: '
                f't=[{tr.x:.3f}, {tr.y:.3f}, {tr.z:.3f}]'
            )
            return True
        except Exception as e:
            self.get_logger().warn(
                f'TF {camera_frame}<-{self._base_frame} not yet available: {e}',
                throttle_duration_sec=5.0,
            )
            return False

    def _callback(self, msg: Odometry):
        if self._t_cb is None:
            if not self._fetch_static_tf(msg.child_frame_id):
                return

        # ── Pose: T_odom_base = T_odom_cam ∘ T_cam_base ──────────────────────
        # pos_base_in_odom = R_odom_cam * t_base_in_cam + pos_cam_in_odom
        # R_odom_base = R_odom_cam * R_cam_base
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        R_oc = Rotation.from_quat([o.x, o.y, o.z, o.w])

        base_pos = R_oc.apply(self._t_cb) + np.array([p.x, p.y, p.z])
        R_odom_base = R_oc * self._R_cb
        q_odom_base = R_odom_base.as_quat()  # [x, y, z, w]

        # ── Twist: rigid-body velocity from camera origin to base_link origin ─
        # v_base_in_cam = v_cam_in_cam + omega_cam × r_base_in_cam
        # then rotate from camera frame to base_link frame via R_cam_base
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular
        omega = np.array([av.x, av.y, av.z])
        v_cam = np.array([lv.x, lv.y, lv.z])

        v_base_in_cam = v_cam + np.cross(omega, self._t_cb)
        base_lv = self._R_cb.apply(v_base_in_cam)
        base_av = self._R_cb.apply(omega)

        # ── Publish ───────────────────────────────────────────────────────────
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id  # 'odom'
        out.child_frame_id = self._base_frame       # 'base_link'

        out.pose.pose.position.x = float(base_pos[0])
        out.pose.pose.position.y = float(base_pos[1])
        out.pose.pose.position.z = float(base_pos[2])
        out.pose.pose.orientation.x = float(q_odom_base[0])
        out.pose.pose.orientation.y = float(q_odom_base[1])
        out.pose.pose.orientation.z = float(q_odom_base[2])
        out.pose.pose.orientation.w = float(q_odom_base[3])

        out.twist.twist.linear.x = float(base_lv[0])
        out.twist.twist.linear.y = float(base_lv[1])
        out.twist.twist.linear.z = float(base_lv[2])
        out.twist.twist.angular.x = float(base_av[0])
        out.twist.twist.angular.y = float(base_av[1])
        out.twist.twist.angular.z = float(base_av[2])

        # Covariance: passed through unchanged (exact rotation of the 6×6 covariance
        # would require building the full Jacobian; for lidar localization odometry
        # hints this approximation is acceptable).
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ZedOdomToBaseLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
