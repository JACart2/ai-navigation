"""Tests for angular velocity derived from localization poses."""

import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped

from navigation.speed_node import SpeedNode


def pose_message(timestamp, yaw):
    """Build a stamped planar pose with the requested yaw."""
    message = PoseWithCovarianceStamped()
    message.header.stamp.sec = int(timestamp)
    message.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def test_pose_callback_calculates_signed_yaw_rate_across_wraparound():
    """Yaw rate must use pose timestamps and the shortest wrapped rotation."""
    node = SimpleNamespace(
        prev_pose=None,
        prev_time=None,
        prev_yaw=None,
        speed_estimate=0.0,
        pose_stream_started=False,
        speed_warning_active=False,
        twist_estimate=None,
        anomaly_logging=lambda message, severity: None,
        pose_time_seconds=SpeedNode.pose_time_seconds,
        quaternion_to_yaw=SpeedNode.quaternion_to_yaw,
        shortest_angular_distance=SpeedNode.shortest_angular_distance,
    )

    SpeedNode.pose_cb(node, pose_message(10.0, math.radians(179.0)))
    SpeedNode.pose_cb(node, pose_message(10.5, math.radians(-179.0)))

    expected_rate = math.radians(2.0) / 0.5
    assert math.isclose(
        node.twist_estimate.twist.angular.z,
        expected_rate,
        rel_tol=1e-6,
    )
    assert node.twist_estimate.header.stamp.sec == 10
    assert node.twist_estimate.header.stamp.nanosec == 500_000_000


def test_quaternion_to_yaw_preserves_turn_direction():
    """Positive and negative yaw orientations must retain their signs."""
    positive = pose_message(1.0, 0.6).pose.pose.orientation
    negative = pose_message(1.0, -0.4).pose.pose.orientation

    assert math.isclose(SpeedNode.quaternion_to_yaw(positive), 0.6)
    assert math.isclose(SpeedNode.quaternion_to_yaw(negative), -0.4)
