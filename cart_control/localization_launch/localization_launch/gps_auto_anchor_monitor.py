"""Monitor whether GPS and MOLA odometry can form a live map anchor."""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


EARTH_RADIUS_M = 6378137.0


@dataclass
class GpsSample:
    time_sec: float
    latitude: float
    longitude: float
    altitude: float
    xy_std: float


@dataclass
class PoseSample:
    time_sec: float
    x: float
    y: float
    z: float
    yaw: float


class GpsAutoAnchorMonitor(Node):
    """Reports when live GPS and MOLA odometry are good enough to anchor."""

    def __init__(self) -> None:
        super().__init__("gps_auto_anchor_monitor")

        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("pose_topic", "/lidar_odometry/pose")
        self.declare_parameter("gps_min_status", 0)
        self.declare_parameter("gps_max_xy_std", 12.0)
        self.declare_parameter("gps_fix_max_age_sec", 5.0)
        self.declare_parameter("min_samples", 5)
        self.declare_parameter("min_move_m", 8.0)
        self.declare_parameter("report_period_sec", 10.0)

        self.fix_topic = str(self.get_parameter("fix_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.gps_min_status = int(self.get_parameter("gps_min_status").value)
        self.gps_max_xy_std = float(self.get_parameter("gps_max_xy_std").value)
        self.gps_fix_max_age_sec = float(
            self.get_parameter("gps_fix_max_age_sec").value
        )
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.min_move_m = float(self.get_parameter("min_move_m").value)
        self.report_period_sec = float(
            self.get_parameter("report_period_sec").value
        )

        self.gps_samples: Deque[GpsSample] = deque(maxlen=80)
        self.anchor_gps: Optional[GpsSample] = None
        self.anchor_pose: Optional[PoseSample] = None
        self.last_pose: Optional[PoseSample] = None
        self.yaw_from_enu: Optional[float] = None
        self.last_report_sec = 0.0

        self.create_subscription(
            NavSatFix,
            self.fix_topic,
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.pose_topic,
            self._pose_callback,
            10,
        )
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            "GPS auto-anchor monitor started; "
            f"fix={self.fix_topic}, pose={self.pose_topic}, "
            f"min_move={self.min_move_m:.1f}m."
        )

    def _fix_callback(self, msg: NavSatFix) -> None:
        sample = self._gps_sample_from_fix(msg)
        if sample is not None:
            self.gps_samples.append(sample)

    def _pose_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.last_pose = PoseSample(
            time_sec=self._now_sec(),
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            yaw=self._yaw_from_quat(pose.orientation),
        )

    def _tick(self) -> None:
        now_sec = self._now_sec()
        gps = self._averaged_recent_gps_sample(now_sec)
        if gps is not None and self.last_pose is not None and self.anchor_gps is None:
            self.anchor_gps = gps
            self.anchor_pose = self.last_pose
            self.get_logger().info(
                "GPS auto-anchor captured: "
                f"lat={gps.latitude:.8f}, lon={gps.longitude:.8f}, "
                f"alt={gps.altitude:.2f}, map=({self.last_pose.x:.2f}, "
                f"{self.last_pose.y:.2f}, {self.last_pose.z:.2f})."
            )

        if (
            gps is not None
            and self.last_pose is not None
            and self.anchor_gps is not None
            and self.anchor_pose is not None
            and self.yaw_from_enu is None
        ):
            east, north = self._gps_delta_from_anchor(gps)
            gps_dist = math.hypot(east, north)
            map_dx = self.last_pose.x - self.anchor_pose.x
            map_dy = self.last_pose.y - self.anchor_pose.y
            map_dist = math.hypot(map_dx, map_dy)
            if gps_dist >= self.min_move_m and map_dist >= self.min_move_m:
                enu_angle = math.atan2(north, east)
                map_angle = math.atan2(map_dy, map_dx)
                self.yaw_from_enu = self._normalize_angle(map_angle - enu_angle)
                self.get_logger().info(
                    "GPS auto-anchor yaw learned: "
                    f"gps_map_yaw_from_enu={self.yaw_from_enu:.4f} rad "
                    f"after gps={gps_dist:.1f}m/map={map_dist:.1f}m motion."
                )

        if now_sec - self.last_report_sec >= self.report_period_sec:
            self.last_report_sec = now_sec
            self._report_status(now_sec, gps)

    def _report_status(self, now_sec: float, gps: Optional[GpsSample]) -> None:
        if gps is None:
            self.get_logger().info(
                "GPS auto-anchor waiting for recent good GPS fixes "
                f"({len(self.gps_samples)}/{self.min_samples} samples)."
            )
            return
        if self.last_pose is None:
            self.get_logger().info(
                "GPS auto-anchor has GPS but is waiting for MOLA odometry."
            )
            return
        if self.anchor_gps is None:
            self.get_logger().info("GPS auto-anchor ready to capture anchor.")
            return
        if self.yaw_from_enu is None:
            east, north = self._gps_delta_from_anchor(gps)
            gps_dist = math.hypot(east, north)
            map_dist = 0.0
            if self.anchor_pose is not None:
                map_dist = math.hypot(
                    self.last_pose.x - self.anchor_pose.x,
                    self.last_pose.y - self.anchor_pose.y,
                )
            self.get_logger().info(
                "GPS auto-anchor captured; yaw waiting for movement "
                f"gps={gps_dist:.1f}m/map={map_dist:.1f}m "
                f"required={self.min_move_m:.1f}m."
            )
            return
        self.get_logger().info(
            "GPS auto-anchor ready for recovery/georef sanity checks: "
            f"gps_map_yaw_from_enu={self.yaw_from_enu:.4f} rad."
        )

    def _gps_sample_from_fix(self, fix: NavSatFix) -> Optional[GpsSample]:
        if fix.status.status < self.gps_min_status:
            return None
        if not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
            return None
        xy_std = self._gps_xy_std(fix)
        if xy_std > self.gps_max_xy_std:
            return None
        altitude = fix.altitude if math.isfinite(fix.altitude) else 0.0
        return GpsSample(
            time_sec=self._now_sec(),
            latitude=fix.latitude,
            longitude=fix.longitude,
            altitude=altitude,
            xy_std=xy_std,
        )

    def _averaged_recent_gps_sample(self, now_sec: float) -> Optional[GpsSample]:
        recent = [
            sample
            for sample in self.gps_samples
            if now_sec - sample.time_sec <= self.gps_fix_max_age_sec
        ]
        if len(recent) < self.min_samples:
            return None
        count = float(len(recent))
        return GpsSample(
            time_sec=max(sample.time_sec for sample in recent),
            latitude=sum(sample.latitude for sample in recent) / count,
            longitude=sum(sample.longitude for sample in recent) / count,
            altitude=sum(sample.altitude for sample in recent) / count,
            xy_std=max(sample.xy_std for sample in recent),
        )

    def _gps_delta_from_anchor(self, sample: GpsSample) -> tuple[float, float]:
        if self.anchor_gps is None:
            return 0.0, 0.0
        lat0 = math.radians(self.anchor_gps.latitude)
        d_lat = math.radians(sample.latitude - self.anchor_gps.latitude)
        d_lon = math.radians(sample.longitude - self.anchor_gps.longitude)
        east = EARTH_RADIUS_M * d_lon * math.cos(lat0)
        north = EARTH_RADIUS_M * d_lat
        return east, north

    def _gps_xy_std(self, fix: NavSatFix) -> float:
        cov = list(fix.position_covariance)
        xy_var = max(cov[0] if len(cov) > 0 else 0.0, cov[4] if len(cov) > 4 else 0.0)
        if not math.isfinite(xy_var) or xy_var <= 0.0:
            return self.gps_max_xy_std
        return math.sqrt(xy_var)

    def _yaw_from_quat(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsAutoAnchorMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
