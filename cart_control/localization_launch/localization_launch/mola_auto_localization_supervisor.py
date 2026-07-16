"""Conservative LiDAR-first MOLA auto-localization supervisor."""

import importlib
import json
import math
import re
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - yaml is available in this workspace
    yaml = None

import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


BOOL_TRUE = {"1", "true", "yes", "on", "active", "enabled"}
BOOL_FALSE = {"0", "false", "no", "off", "inactive", "disabled"}


@dataclass
class PoseSample:
    time_sec: float
    x: float
    y: float
    z: float
    yaw: float
    msg: Odometry




@dataclass
class GpsSample:
    time_sec: float
    latitude: float
    longitude: float
    altitude: float
    xy_std: float


@dataclass
class GpsMapAnchor:
    gps: GpsSample
    map_x: float
    map_y: float
    map_z: float
    map_yaw: float
    yaw_from_enu: Optional[float] = None


class MolaAutoLocalizationSupervisor(Node):
    """Watch MOLA health and request cautious relocalization when needed."""

    EARTH_RADIUS_M = 6378137.0

    def __init__(self) -> None:
        super().__init__("mola_auto_localization_supervisor")

        self._declare_parameters()
        self._read_parameters()

        self.node_start_sec = self._now_sec()
        self.first_cloud_time: Optional[float] = None
        self.last_cloud_time: Optional[float] = None
        self.last_cloud_points = 0
        self.last_pose_time: Optional[float] = None
        self.last_pose_sample: Optional[PoseSample] = None
        self.last_good_pose: Optional[PoseWithCovarianceStamped] = None
        self.last_good_pose_time: Optional[float] = None
        self.last_pcl_pose_time: Optional[float] = None
        self.last_gps_fix: Optional[NavSatFix] = None
        self.last_gps_fix_time: Optional[float] = None
        self.gps_samples: Deque[GpsSample] = deque(maxlen=60)
        self.gps_map_anchor: Optional[GpsMapAnchor] = None
        self.logged_first_gps_fix = False
        self.logged_gps_anchor_wait = False
        self.last_diag_time: Optional[float] = None
        self.last_diag_fields: Dict[str, Any] = {}
        self.bad_diag_since: Optional[float] = None
        self.bad_diag_reasons: List[str] = []
        self.localization_good_since: Optional[float] = None

        self.logged_cloud_wait = False
        self.logged_cloud_healthy = False
        self.logged_pose_wait = False
        self.logged_first_pose = False
        self.logged_healthy = False

        self.lost_active = False
        self.loss_event_id = 0
        self.current_loss_reasons: List[str] = []
        self.recovery_attempts_this_event = 0
        self.startup_attempts = 0
        self.startup_relocalize_complete = False
        self.last_relocalize_time: Optional[float] = None
        self.pending_relocalize_future = None
        self.pending_relocalize_start: Optional[float] = None
        self.pending_relocalize_context = ""

        self.relocalize_client = None
        self.relocalize_srv_type = None
        self.relocalize_srv_type_name = ""
        self._last_log_times: Dict[str, float] = {}

        self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.mola_pose_topic,
            self._pose_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pcl_pose_topic,
            self._pcl_pose_callback,
            10,
        )
        self.create_subscription(
            String,
            self.diagnostics_topic,
            self._diagnostics_callback,
            10,
        )
        if self.enable_gps_recovery:
            self.create_subscription(
                NavSatFix,
                self.gps_fix_topic,
                self._gps_fix_callback,
                qos_profile_sensor_data,
            )
        self.create_timer(self.check_period_sec, self._tick)

        self.get_logger().info(
            "MOLA auto-localization supervisor started; "
            f"cloud={self.cloud_topic}, pose={self.mola_pose_topic}, "
            f"diagnostics={self.diagnostics_topic}, "
            f"relocalize_service={self.relocalize_service}"
        )
        if self.enable_gps_recovery:
            if self.enable_gps_auto_anchor:
                self.get_logger().info(
                    "GPS-assisted recovery enabled with auto-anchor learning; "
                    "GPS will become a recovery hint after MOLA is healthy "
                    "and the cart drives far enough to estimate map yaw."
                )
            elif self._gps_origin_configured():
                self.get_logger().info(
                    "GPS-assisted recovery enabled; /fix will be used as a "
                    "broad map-frame suggestion only after localization is lost."
                )
            else:
                self.get_logger().warn(
                    "GPS-assisted recovery is enabled but map GPS origin is "
                    "not configured and auto-anchor is disabled; GPS recovery "
                    "hints will be ignored."
                )
        self.get_logger().info(
            "Waiting for LiDAR before requesting startup relocalization."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("cloud_topic", "/velodyne_points")
        self.declare_parameter("mola_pose_topic", "/lidar_odometry/pose")
        self.declare_parameter("pcl_pose_topic", "/pcl_pose")
        self.declare_parameter(
            "diagnostics_topic",
            "/mola_diagnostics/lidar_odom/status",
        )
        self.declare_parameter("relocalize_service", "/relocalize_near_pose")
        self.declare_parameter("relocalize_service_type", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("check_period_sec", 1.0)
        self.declare_parameter("min_cloud_points_for_relocalize", 2000)
        self.declare_parameter("cloud_stale_timeout_sec", 2.0)
        self.declare_parameter("pose_stale_timeout_sec", 3.0)
        self.declare_parameter("min_good_pose_age_sec", 3.0)
        self.declare_parameter("bad_icp_quality_threshold", 0.2)
        self.declare_parameter("bad_dropped_frames_threshold", 0.4)
        self.declare_parameter("bad_diagnostics_grace_sec", 5.0)
        self.declare_parameter("max_reasonable_speed_mps", 8.0)
        self.declare_parameter("max_reasonable_yaw_rate_radps", 3.0)
        self.declare_parameter("max_pose_jump_m", 8.0)
        self.declare_parameter("max_yaw_jump_rad", 2.5)
        self.declare_parameter("relocalize_cooldown_sec", 15.0)
        self.declare_parameter("startup_relocalize_delay_sec", 5.0)
        self.declare_parameter("startup_pose_x", 0.0)
        self.declare_parameter("startup_pose_y", 0.0)
        self.declare_parameter("startup_pose_z", 0.0)
        self.declare_parameter("startup_pose_yaw", 0.0)
        self.declare_parameter("startup_xy_std", 8.0)
        self.declare_parameter("startup_yaw_std", 1.57)
        self.declare_parameter("recovery_xy_std", 4.0)
        self.declare_parameter("recovery_yaw_std", 0.8)
        self.declare_parameter("enable_startup_relocalize", True)
        self.declare_parameter("enable_recovery_relocalize", True)
        self.declare_parameter("enable_gps_recovery", False)
        self.declare_parameter("enable_gps_auto_anchor", True)
        self.declare_parameter("gps_fix_topic", "/fix")
        self.declare_parameter("gps_fix_max_age_sec", 5.0)
        self.declare_parameter("gps_min_status", 0)
        self.declare_parameter("gps_max_xy_std", 25.0)
        self.declare_parameter("gps_min_xy_std", 6.0)
        self.declare_parameter("gps_z_std", 30.0)
        self.declare_parameter("gps_yaw_std", 3.14)
        self.declare_parameter("gps_auto_anchor_min_samples", 5)
        self.declare_parameter("gps_auto_anchor_min_move_m", 8.0)
        self.declare_parameter("gps_auto_anchor_max_xy_std", 12.0)
        self.declare_parameter("gps_map_origin_lat", 0.0)
        self.declare_parameter("gps_map_origin_lon", 0.0)
        self.declare_parameter("gps_map_origin_alt", 0.0)
        self.declare_parameter("gps_map_yaw_from_enu", 0.0)
        self.declare_parameter("max_relocalize_attempts_per_event", 3)
        self.declare_parameter("service_wait_timeout_sec", 0.05)
        self.declare_parameter("relocalize_response_timeout_sec", 10.0)
        self.declare_parameter("debug", False)

    def _read_parameters(self) -> None:
        self.cloud_topic = self._param_str("cloud_topic")
        self.mola_pose_topic = self._param_str("mola_pose_topic")
        self.pcl_pose_topic = self._param_str("pcl_pose_topic")
        self.diagnostics_topic = self._param_str("diagnostics_topic")
        self.relocalize_service = self._param_str("relocalize_service")
        self.relocalize_service_type_param = self._param_str(
            "relocalize_service_type"
        ).strip()
        self.map_frame = self._param_str("map_frame")
        self.check_period_sec = max(0.1, self._param_float("check_period_sec"))
        self.min_cloud_points_for_relocalize = self._param_int(
            "min_cloud_points_for_relocalize"
        )
        self.cloud_stale_timeout_sec = self._param_float(
            "cloud_stale_timeout_sec"
        )
        self.pose_stale_timeout_sec = self._param_float(
            "pose_stale_timeout_sec"
        )
        self.min_good_pose_age_sec = self._param_float(
            "min_good_pose_age_sec"
        )
        self.bad_icp_quality_threshold = self._param_float(
            "bad_icp_quality_threshold"
        )
        self.bad_dropped_frames_threshold = self._param_float(
            "bad_dropped_frames_threshold"
        )
        self.bad_diagnostics_grace_sec = self._param_float(
            "bad_diagnostics_grace_sec"
        )
        self.max_reasonable_speed_mps = self._param_float(
            "max_reasonable_speed_mps"
        )
        self.max_reasonable_yaw_rate_radps = self._param_float(
            "max_reasonable_yaw_rate_radps"
        )
        self.max_pose_jump_m = self._param_float("max_pose_jump_m")
        self.max_yaw_jump_rad = self._param_float("max_yaw_jump_rad")
        self.relocalize_cooldown_sec = self._param_float(
            "relocalize_cooldown_sec"
        )
        self.startup_relocalize_delay_sec = self._param_float(
            "startup_relocalize_delay_sec"
        )
        self.startup_pose_x = self._param_float("startup_pose_x")
        self.startup_pose_y = self._param_float("startup_pose_y")
        self.startup_pose_z = self._param_float("startup_pose_z")
        self.startup_pose_yaw = self._param_float("startup_pose_yaw")
        self.startup_xy_std = self._param_float("startup_xy_std")
        self.startup_yaw_std = self._param_float("startup_yaw_std")
        self.recovery_xy_std = self._param_float("recovery_xy_std")
        self.recovery_yaw_std = self._param_float("recovery_yaw_std")
        self.enable_startup_relocalize = self._param_bool(
            "enable_startup_relocalize"
        )
        self.enable_recovery_relocalize = self._param_bool(
            "enable_recovery_relocalize"
        )
        self.enable_gps_recovery = self._param_bool("enable_gps_recovery")
        self.enable_gps_auto_anchor = self._param_bool("enable_gps_auto_anchor")
        self.gps_fix_topic = self._param_str("gps_fix_topic")
        self.gps_fix_max_age_sec = self._param_float("gps_fix_max_age_sec")
        self.gps_min_status = self._param_int("gps_min_status")
        self.gps_max_xy_std = self._param_float("gps_max_xy_std")
        self.gps_min_xy_std = self._param_float("gps_min_xy_std")
        self.gps_z_std = self._param_float("gps_z_std")
        self.gps_yaw_std = self._param_float("gps_yaw_std")
        self.gps_auto_anchor_min_samples = max(1, self._param_int("gps_auto_anchor_min_samples"))
        self.gps_auto_anchor_min_move_m = self._param_float("gps_auto_anchor_min_move_m")
        self.gps_auto_anchor_max_xy_std = self._param_float("gps_auto_anchor_max_xy_std")
        self.gps_map_origin_lat = self._param_float("gps_map_origin_lat")
        self.gps_map_origin_lon = self._param_float("gps_map_origin_lon")
        self.gps_map_origin_alt = self._param_float("gps_map_origin_alt")
        self.gps_map_yaw_from_enu = self._param_float("gps_map_yaw_from_enu")
        self.max_relocalize_attempts_per_event = max(
            1,
            self._param_int("max_relocalize_attempts_per_event"),
        )
        self.service_wait_timeout_sec = max(
            0.0,
            self._param_float("service_wait_timeout_sec"),
        )
        self.relocalize_response_timeout_sec = self._param_float(
            "relocalize_response_timeout_sec"
        )
        self.debug = self._param_bool("debug")

    def _param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _param_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _param_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in BOOL_TRUE

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self.last_cloud_time = self._now_sec()
        if self.first_cloud_time is None:
            self.first_cloud_time = self.last_cloud_time
        self.last_cloud_points = int(msg.width) * int(msg.height)

        if (
            not self.logged_cloud_healthy
            and self.last_cloud_points >= self.min_cloud_points_for_relocalize
        ):
            self.logged_cloud_healthy = True
            self.get_logger().info(
                "LiDAR healthy; latest cloud has "
                f"{self.last_cloud_points} points."
            )

    def _pose_callback(self, msg: Odometry) -> None:
        now_sec = self._now_sec()
        sample = self._sample_from_odom(msg, now_sec)
        reasons = self._pose_sanity_reasons(sample)

        self.last_pose_time = now_sec
        self.last_pose_sample = sample

        if not self.logged_first_pose:
            self.logged_first_pose = True
            self.get_logger().info("Received first MOLA pose.")

        if reasons:
            self._mark_lost(reasons)
            return

        if not self.lost_active:
            self.last_good_pose = self._pose_msg_from_sample(
                sample,
                self.recovery_xy_std,
                self.recovery_yaw_std,
            )
            self.last_good_pose_time = now_sec

    def _pcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        del msg
        self.last_pcl_pose_time = self._now_sec()
        self._log_throttled(
            "info",
            "first_pcl_pose",
            3600.0,
            "Observed /pcl_pose relay; using it as secondary visibility only.",
        )

    def _gps_fix_callback(self, msg: NavSatFix) -> None:
        now_sec = self._now_sec()
        self.last_gps_fix = msg
        self.last_gps_fix_time = now_sec
        if not self.logged_first_gps_fix:
            self.logged_first_gps_fix = True
            self.get_logger().info(
                f"Received first GPS fix on {self.gps_fix_topic}."
            )

        sample = self._gps_sample_from_fix(msg, now_sec)
        if sample is not None:
            self.gps_samples.append(sample)

    def _diagnostics_callback(self, msg: String) -> None:
        now_sec = self._now_sec()
        self.last_diag_time = now_sec
        fields = self._parse_diagnostics_text(msg.data)
        self.last_diag_fields = fields

        reasons: List[str] = []
        active = self._diag_value(fields, "active")
        if active is False or active == 0.0:
            reasons.append("MOLA diagnostics report inactive")

        icp_quality = self._diag_float(fields, "icp_quality")
        if (
            icp_quality is not None
            and icp_quality < self.bad_icp_quality_threshold
        ):
            reasons.append(
                "ICP quality "
                f"{icp_quality:.3f} below "
                f"{self.bad_icp_quality_threshold:.3f}"
            )

        dropped = self._diag_float(fields, "dropped_frames_ratio")
        if (
            dropped is not None
            and dropped > self.bad_dropped_frames_threshold
        ):
            reasons.append(
                "dropped frame ratio "
                f"{dropped:.3f} above "
                f"{self.bad_dropped_frames_threshold:.3f}"
            )

        if reasons:
            if self.bad_diag_since is None:
                self.bad_diag_since = now_sec
            self.bad_diag_reasons = reasons
        else:
            self.bad_diag_since = None
            self.bad_diag_reasons = []

        if self.debug and fields:
            self.get_logger().debug(f"Parsed MOLA diagnostics: {fields}")

    def _tick(self) -> None:
        now_sec = self._now_sec()
        self._check_pending_relocalize(now_sec)
        self._log_waiting_states(now_sec)

        reasons = self._timer_loss_reasons(now_sec)
        if reasons:
            self._mark_lost(reasons)
        else:
            self._update_healthy_state(now_sec)

        self._maybe_startup_relocalize(now_sec)
        self._maybe_recovery_relocalize(now_sec)

    def _log_waiting_states(self, now_sec: float) -> None:
        if self.last_cloud_time is None:
            self._log_throttled(
                "info",
                "waiting_lidar",
                10.0,
                f"Waiting for LiDAR cloud on {self.cloud_topic}.",
            )
            return

        cloud_age = now_sec - self.last_cloud_time
        if cloud_age > self.cloud_stale_timeout_sec:
            self._log_throttled(
                "warn",
                "cloud_stale",
                5.0,
                f"LiDAR cloud stale for {cloud_age:.1f}s.",
            )

        if self.last_pose_time is None:
            self._log_throttled(
                "info",
                "waiting_pose",
                10.0,
                f"Waiting for MOLA pose on {self.mola_pose_topic}.",
            )

    def _timer_loss_reasons(self, now_sec: float) -> List[str]:
        reasons: List[str] = []

        cloud_alive = self._cloud_alive(now_sec)
        if cloud_alive and self.last_pose_time is None:
            cloud_stream_age = now_sec - (self.first_cloud_time or now_sec)
            if cloud_stream_age >= self.pose_stale_timeout_sec:
                reasons.append(
                    "LiDAR alive but no MOLA pose has been received"
                )
        elif cloud_alive and self.last_pose_time is not None:
            pose_age = now_sec - self.last_pose_time
            if pose_age > self.pose_stale_timeout_sec:
                reasons.append(f"MOLA pose stale for {pose_age:.1f}s")

        if self.bad_diag_since is not None:
            bad_age = now_sec - self.bad_diag_since
            if bad_age >= self.bad_diagnostics_grace_sec:
                reasons.extend(self.bad_diag_reasons)

        return reasons

    def _update_healthy_state(self, now_sec: float) -> None:
        if not self._pose_recent(now_sec):
            self.localization_good_since = None
            self.logged_healthy = False
            return

        if self.bad_diag_since is not None:
            self.localization_good_since = None
            self.logged_healthy = False
            return

        if self.localization_good_since is None:
            self.localization_good_since = now_sec
            return

        good_age = now_sec - self.localization_good_since
        if good_age < self.min_good_pose_age_sec:
            return

        if self.lost_active:
            self.get_logger().info(
                "Localization healthy again after "
                f"{good_age:.1f}s of stable pose updates."
            )
            self.lost_active = False
            self.current_loss_reasons = []
            self.recovery_attempts_this_event = 0

        if not self.logged_healthy:
            self.get_logger().info("Localization healthy.")
            self.logged_healthy = True

        self._maybe_update_gps_auto_anchor(now_sec)

        if self.last_pose_sample is not None:
            self.last_good_pose = self._pose_msg_from_sample(
                self.last_pose_sample,
                self.recovery_xy_std,
                self.recovery_yaw_std,
            )
            self.last_good_pose_time = now_sec

    def _maybe_startup_relocalize(self, now_sec: float) -> None:
        if not self.enable_startup_relocalize:
            return
        if self.startup_relocalize_complete:
            return
        if self.startup_attempts >= self.max_relocalize_attempts_per_event:
            return
        if now_sec - self.node_start_sec < self.startup_relocalize_delay_sec:
            return
        if not self._cloud_ready(now_sec):
            self._log_throttled(
                "info",
                "startup_wait_cloud",
                5.0,
                "Startup relocalization waiting for a recent, dense "
                "LiDAR cloud.",
            )
            return

        requested = self._request_relocalize(
            "startup",
            "startup relocalization",
            use_startup_std=True,
        )
        if requested:
            self.startup_attempts += 1

    def _maybe_recovery_relocalize(self, now_sec: float) -> None:
        if not self.enable_recovery_relocalize or not self.lost_active:
            return
        if self.recovery_attempts_this_event >= (
            self.max_relocalize_attempts_per_event
        ):
            self._log_throttled(
                "warn",
                "recovery_retry_limit",
                20.0,
                "Recovery relocalization retry limit reached for this "
                "loss event.",
            )
            return
        if not self._cloud_ready(now_sec):
            self._log_throttled(
                "info",
                "recovery_wait_cloud",
                5.0,
                "Recovery relocalization waiting for a recent, dense "
                "LiDAR cloud.",
            )
            return

        requested = self._request_relocalize(
            "recovery",
            "localization suspected lost",
            use_startup_std=False,
        )
        if requested:
            self.recovery_attempts_this_event += 1

    def _request_relocalize(
        self,
        kind: str,
        reason: str,
        use_startup_std: bool,
    ) -> bool:
        now_sec = self._now_sec()
        if self.pending_relocalize_future is not None:
            self._log_throttled(
                "info",
                "relocalize_pending",
                5.0,
                "Relocalization request already pending.",
            )
            return False

        if self.last_relocalize_time is not None:
            cooldown_left = (
                self.relocalize_cooldown_sec
                - (now_sec - self.last_relocalize_time)
            )
            if cooldown_left > 0.0:
                self._log_throttled(
                    "info",
                    "relocalize_cooldown",
                    5.0,
                    f"Relocalization cooldown active for "
                    f"{cooldown_left:.1f}s.",
                )
                return False

        client = self._ensure_relocalize_client()
        if client is None:
            return False

        service_ready = client.wait_for_service(
            timeout_sec=self.service_wait_timeout_sec
        )
        if not service_ready:
            self._log_throttled(
                "warn",
                "relocalize_service_wait",
                5.0,
                f"Waiting for relocalization service "
                f"{self.relocalize_service}.",
            )
            return False

        pose_msg, pose_source, suggested_xy_std, suggested_yaw_std = (
            self._select_relocalize_pose(kind)
        )
        xy_std = suggested_xy_std or (
            self.startup_xy_std if use_startup_std else self.recovery_xy_std
        )
        yaw_std = suggested_yaw_std or (
            self.startup_yaw_std if use_startup_std else self.recovery_yaw_std
        )

        request = self.relocalize_srv_type.Request()
        assigned = self._populate_relocalize_request(
            request,
            pose_msg,
            xy_std,
            yaw_std,
        )

        if self.debug:
            self.get_logger().debug(
                "Prepared relocalization request fields: "
                f"{', '.join(assigned) if assigned else '<none>'}"
            )

        self.pending_relocalize_future = client.call_async(request)
        self.pending_relocalize_start = now_sec
        self.pending_relocalize_context = kind
        self.pending_relocalize_future.add_done_callback(
            lambda future: self._relocalize_done(future, kind)
        )
        self.last_relocalize_time = now_sec

        self.get_logger().warn(
            "Relocalization requested: "
            f"{reason}; using {pose_source} pose; "
            f"service_type={self.relocalize_srv_type_name}; "
            f"xy_std={xy_std:.2f}, yaw_std={yaw_std:.2f}."
        )
        return True

    def _ensure_relocalize_client(self):
        if self.relocalize_client is not None:
            return self.relocalize_client

        type_names = self._relocalize_type_candidates()
        for type_name in type_names:
            srv_type = self._import_ros_interface(type_name, "srv")
            if srv_type is None:
                continue
            self.relocalize_srv_type = srv_type
            self.relocalize_srv_type_name = type_name
            self.relocalize_client = self.create_client(
                srv_type,
                self.relocalize_service,
            )
            self.get_logger().info(
                "Relocalization service client ready for "
                f"{self.relocalize_service} ({type_name})."
            )
            return self.relocalize_client

        self._log_throttled(
            "warn",
            "relocalize_type_wait",
            10.0,
            "Could not discover/import relocalization service type for "
            f"{self.relocalize_service}; supervisor will keep waiting.",
        )
        return None

    def _relocalize_type_candidates(self) -> List[str]:
        candidates: List[str] = []
        if self.relocalize_service_type_param:
            candidates.append(self.relocalize_service_type_param)

        service_names_and_types = self.get_service_names_and_types()
        wanted = self.relocalize_service
        wanted_base = "/" + wanted.strip("/").split("/")[-1]
        for service_name, type_names in service_names_and_types:
            if service_name == wanted or service_name.endswith(wanted_base):
                candidates.extend(type_names)

        deduped: List[str] = []
        for type_name in candidates:
            if type_name not in deduped:
                deduped.append(type_name)
        return deduped

    def _import_ros_interface(self, type_name: str, submodule: str):
        parts = type_name.split("/")
        if len(parts) == 3:
            package, middle, class_name = parts
            if middle not in (submodule, f"{submodule}s"):
                return None
        elif len(parts) == 2:
            package, class_name = parts
        else:
            return None

        module_name = f"{package}.{submodule}"
        try:
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except (ImportError, AttributeError) as exc:
            self._log_throttled(
                "warn",
                f"import_{type_name}",
                30.0,
                f"Unable to import ROS interface {type_name}: {exc}",
            )
            return None

    def _populate_relocalize_request(
        self,
        request: Any,
        pose_msg: PoseWithCovarianceStamped,
        xy_std: float,
        yaw_std: float,
    ) -> List[str]:
        assigned: List[str] = []
        field_types = self._field_types(request)
        yaw = self._yaw_from_pose(pose_msg.pose.pose)

        for field_name, field_type in field_types.items():
            lower_type = field_type.lower()
            if "posewithcovariancestamped" in lower_type:
                if self._try_setattr(request, field_name, pose_msg):
                    assigned.append(field_name)
            elif "posewithcovariance" in lower_type:
                if self._try_setattr(request, field_name, pose_msg.pose):
                    assigned.append(field_name)
            elif "posestamped" in lower_type:
                if self._try_setattr(
                    request,
                    field_name,
                    self._pose_stamped_from_cov(pose_msg),
                ):
                    assigned.append(field_name)
            elif lower_type.endswith("/pose") or lower_type.endswith(".pose"):
                if self._try_setattr(request, field_name, pose_msg.pose.pose):
                    assigned.append(field_name)

        numeric_values = self._numeric_request_values(
            pose_msg,
            xy_std,
            yaw_std,
        )
        for field_name, value in numeric_values.items():
            if hasattr(request, field_name):
                if self._try_setattr(request, field_name, value):
                    assigned.append(field_name)

        if hasattr(request, "covariance"):
            covariance = self._covariance_from_std(xy_std, yaw_std)
            if self._try_setattr(request, "covariance", covariance):
                assigned.append("covariance")

        return assigned

    def _numeric_request_values(
        self,
        pose_msg: PoseWithCovarianceStamped,
        xy_std: float,
        yaw_std: float,
    ) -> Dict[str, float]:
        pose = pose_msg.pose.pose
        yaw = self._yaw_from_pose(pose)
        return {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": yaw,
            "heading": yaw,
            "initial_x": pose.position.x,
            "initial_y": pose.position.y,
            "initial_z": pose.position.z,
            "initial_yaw": yaw,
            "xy_std": xy_std,
            "yaw_std": yaw_std,
            "std_xy": xy_std,
            "std_yaw": yaw_std,
            "sigma_xy": xy_std,
            "sigma_yaw": yaw_std,
            "x_std": xy_std,
            "y_std": xy_std,
            "z_std": max(0.1, xy_std),
            "roll_std": math.pi,
            "pitch_std": math.pi,
        }

    def _field_types(self, msg: Any) -> Dict[str, str]:
        if hasattr(msg, "get_fields_and_field_types"):
            return msg.get_fields_and_field_types()
        return {}

    def _try_setattr(self, obj: Any, name: str, value: Any) -> bool:
        try:
            setattr(obj, name, value)
            return True
        except (AssertionError, AttributeError, TypeError, ValueError):
            return False

    def _relocalize_done(self, future: Any, kind: str) -> None:
        self.pending_relocalize_future = None
        self.pending_relocalize_start = None
        self.pending_relocalize_context = ""

        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - depends on ROS graph
            self.get_logger().warn(f"Relocalization request failed: {exc}")
            return

        success = self._response_success(response)
        message = self._response_message(response)
        if success:
            self.get_logger().info(
                "Relocalization request succeeded"
                f"{': ' + message if message else '.'}"
            )
            if kind == "startup":
                self.startup_relocalize_complete = True
        else:
            self.get_logger().warn(
                "Relocalization request returned failure"
                f"{': ' + message if message else '.'}"
            )

    def _check_pending_relocalize(self, now_sec: float) -> None:
        if self.pending_relocalize_future is None:
            return
        if self.pending_relocalize_start is None:
            return
        age = now_sec - self.pending_relocalize_start
        if age <= self.relocalize_response_timeout_sec:
            return

        self.get_logger().warn(
            "Relocalization response timed out after "
            f"{age:.1f}s; backing off before another request."
        )
        self.pending_relocalize_future = None
        self.pending_relocalize_start = None
        self.pending_relocalize_context = ""

    def _response_success(self, response: Any) -> bool:
        for name in ("success", "ok", "accepted", "result"):
            if hasattr(response, name):
                value = getattr(response, name)
                if isinstance(value, bool):
                    return value
        return True

    def _response_message(self, response: Any) -> str:
        for name in ("message", "status", "reason", "error_message"):
            if hasattr(response, name):
                value = getattr(response, name)
                if value:
                    return str(value)
        return ""

    def _select_relocalize_pose(
        self,
        kind: str,
    ) -> Tuple[PoseWithCovarianceStamped, str, Optional[float], Optional[float]]:
        if kind == "recovery":
            gps_pose = self._gps_recovery_pose_msg()
            if gps_pose is not None:
                pose_msg, xy_std, yaw_std = gps_pose
                return pose_msg, "GPS recovery hint", xy_std, yaw_std

        if self.last_good_pose is not None:
            return self.last_good_pose, "last known good", None, None
        return self._startup_pose_msg(), "configured startup", None, None

    def _maybe_update_gps_auto_anchor(self, now_sec: float) -> None:
        if not (self.enable_gps_recovery and self.enable_gps_auto_anchor):
            return
        if self.lost_active or self.last_pose_sample is None:
            return
        if self.last_good_pose_time is None:
            return

        sample = self._averaged_recent_gps_sample(now_sec)
        if sample is None:
            self._log_throttled(
                "info",
                "gps_auto_anchor_wait_samples",
                10.0,
                "GPS auto-anchor waiting for several recent good fixes.",
            )
            return

        pose = self.last_pose_sample
        if self.gps_map_anchor is None:
            self.gps_map_anchor = GpsMapAnchor(
                gps=sample,
                map_x=pose.x,
                map_y=pose.y,
                map_z=pose.z,
                map_yaw=pose.yaw,
            )
            self.get_logger().info(
                "GPS auto-anchor captured at healthy MOLA pose: "
                f"lat={sample.latitude:.8f}, lon={sample.longitude:.8f}, "
                f"alt={sample.altitude:.2f}, map=({pose.x:.2f}, "
                f"{pose.y:.2f}, {pose.z:.2f}). Waiting for "
                f"{self.gps_auto_anchor_min_move_m:.1f}m of healthy movement "
                "to learn map yaw."
            )
            return

        if self.gps_map_anchor.yaw_from_enu is not None:
            return

        east, north, _ = self._gps_delta_from_anchor(sample)
        gps_dist = math.hypot(east, north)
        map_dx = pose.x - self.gps_map_anchor.map_x
        map_dy = pose.y - self.gps_map_anchor.map_y
        map_dist = math.hypot(map_dx, map_dy)
        min_move = self.gps_auto_anchor_min_move_m
        if gps_dist < min_move or map_dist < min_move:
            self._log_throttled(
                "info",
                "gps_auto_anchor_wait_motion",
                10.0,
                "GPS auto-anchor captured; waiting for enough healthy "
                f"motion to learn yaw (gps={gps_dist:.1f}m, "
                f"map={map_dist:.1f}m, required={min_move:.1f}m).",
            )
            return

        enu_angle = math.atan2(north, east)
        map_angle = math.atan2(map_dy, map_dx)
        self.gps_map_anchor.yaw_from_enu = self._normalize_angle(
            map_angle - enu_angle
        )
        self.get_logger().info(
            "GPS auto-anchor yaw learned: "
            f"gps_map_yaw_from_enu={self.gps_map_anchor.yaw_from_enu:.4f} rad "
            f"after gps={gps_dist:.1f}m/map={map_dist:.1f}m movement."
        )

    def _gps_sample_from_fix(
        self,
        fix: NavSatFix,
        now_sec: float,
    ) -> Optional[GpsSample]:
        if fix.status.status < self.gps_min_status:
            return None
        if not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
            return None
        xy_std = self._gps_xy_std(fix)
        if xy_std > self.gps_auto_anchor_max_xy_std:
            return None
        altitude = fix.altitude if math.isfinite(fix.altitude) else 0.0
        return GpsSample(
            time_sec=now_sec,
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
            and sample.xy_std <= self.gps_auto_anchor_max_xy_std
        ]
        if len(recent) < self.gps_auto_anchor_min_samples:
            return None

        count = float(len(recent))
        return GpsSample(
            time_sec=max(sample.time_sec for sample in recent),
            latitude=sum(sample.latitude for sample in recent) / count,
            longitude=sum(sample.longitude for sample in recent) / count,
            altitude=sum(sample.altitude for sample in recent) / count,
            xy_std=max(sample.xy_std for sample in recent),
        )

    def _gps_auto_anchor_recovery_pose_msg(
        self,
    ) -> Optional[Tuple[PoseWithCovarianceStamped, float, float]]:
        if self.gps_map_anchor is None:
            self._log_throttled(
                "info",
                "gps_auto_anchor_not_captured",
                10.0,
                "GPS recovery waiting for a healthy MOLA pose to capture "
                "the auto-anchor.",
            )
            return None
        if self.gps_map_anchor.yaw_from_enu is None:
            self._log_throttled(
                "info",
                "gps_auto_anchor_no_yaw",
                10.0,
                "GPS recovery waiting for auto-anchor yaw to be learned "
                "from healthy motion.",
            )
            return None
        if self.last_gps_fix is None or self.last_gps_fix_time is None:
            return None

        now_sec = self._now_sec()
        age = now_sec - self.last_gps_fix_time
        if age > self.gps_fix_max_age_sec:
            self._log_throttled(
                "warn",
                "gps_auto_anchor_stale",
                5.0,
                f"GPS auto-anchor hint stale for {age:.1f}s; ignoring GPS.",
            )
            return None

        fix_sample = self._gps_sample_from_fix(self.last_gps_fix, now_sec)
        if fix_sample is None:
            self._log_throttled(
                "warn",
                "gps_auto_anchor_bad_fix",
                5.0,
                "GPS auto-anchor hint ignored because the latest fix is "
                "not good enough.",
            )
            return None

        east, north, up = self._gps_delta_from_anchor(fix_sample)
        yaw = self.gps_map_anchor.yaw_from_enu
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_dx = cos_yaw * east - sin_yaw * north
        map_dy = sin_yaw * east + cos_yaw * north

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = self.gps_map_anchor.map_x + map_dx
        msg.pose.pose.position.y = self.gps_map_anchor.map_y + map_dy
        msg.pose.pose.position.z = self.gps_map_anchor.map_z + up
        msg.pose.pose.orientation = self._quaternion_from_yaw(
            self.gps_map_anchor.map_yaw
        )
        msg.pose.covariance = self._covariance_from_std(
            fix_sample.xy_std,
            self.gps_yaw_std,
            z_std=self.gps_z_std,
        )
        return msg, fix_sample.xy_std, self.gps_yaw_std

    def _gps_delta_from_anchor(self, sample: GpsSample) -> Tuple[float, float, float]:
        if self.gps_map_anchor is None:
            return 0.0, 0.0, 0.0
        anchor = self.gps_map_anchor.gps
        lat0 = math.radians(anchor.latitude)
        d_lat = math.radians(sample.latitude - anchor.latitude)
        d_lon = math.radians(sample.longitude - anchor.longitude)
        east = self.EARTH_RADIUS_M * d_lon * math.cos(lat0)
        north = self.EARTH_RADIUS_M * d_lat
        up = sample.altitude - anchor.altitude
        return east, north, up

    def _gps_recovery_pose_msg(
        self,
    ) -> Optional[Tuple[PoseWithCovarianceStamped, float, float]]:
        if not self.enable_gps_recovery:
            return None
        if self.enable_gps_auto_anchor:
            anchored_pose = self._gps_auto_anchor_recovery_pose_msg()
            if anchored_pose is not None:
                return anchored_pose
            if not self._gps_origin_configured():
                return None
        elif not self._gps_origin_configured():
            return None
        if self.last_gps_fix is None or self.last_gps_fix_time is None:
            self._log_throttled(
                "info",
                "gps_recovery_wait_fix",
                10.0,
                "GPS recovery waiting for a NavSatFix.",
            )
            return None

        now_sec = self._now_sec()
        age = now_sec - self.last_gps_fix_time
        if age > self.gps_fix_max_age_sec:
            self._log_throttled(
                "warn",
                "gps_recovery_stale",
                5.0,
                f"GPS recovery hint stale for {age:.1f}s; ignoring GPS.",
            )
            return None

        fix = self.last_gps_fix
        if fix.status.status < self.gps_min_status:
            self._log_throttled(
                "warn",
                "gps_recovery_bad_status",
                5.0,
                "GPS recovery hint ignored because fix status "
                f"{fix.status.status} is below {self.gps_min_status}.",
            )
            return None
        if not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
            return None

        xy_std = self._gps_xy_std(fix)
        if xy_std > self.gps_max_xy_std:
            self._log_throttled(
                "warn",
                "gps_recovery_bad_covariance",
                5.0,
                "GPS recovery hint ignored because estimated xy std "
                f"{xy_std:.1f}m exceeds {self.gps_max_xy_std:.1f}m.",
            )
            return None

        x, y, z = self._gps_fix_to_map_xyz(fix)
        yaw = self.startup_pose_yaw
        if self.last_good_pose is not None:
            yaw = self._yaw_from_pose(self.last_good_pose.pose.pose)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation = self._quaternion_from_yaw(yaw)
        msg.pose.covariance = self._covariance_from_std(
            xy_std,
            self.gps_yaw_std,
            z_std=self.gps_z_std,
        )
        return msg, xy_std, self.gps_yaw_std

    def _gps_origin_configured(self) -> bool:
        return (
            abs(self.gps_map_origin_lat) > 1e-9
            or abs(self.gps_map_origin_lon) > 1e-9
        )

    def _gps_xy_std(self, fix: NavSatFix) -> float:
        cov = list(fix.position_covariance)
        xy_var = max(cov[0] if len(cov) > 0 else 0.0, cov[4] if len(cov) > 4 else 0.0)
        if not math.isfinite(xy_var) or xy_var <= 0.0:
            return self.gps_max_xy_std
        return max(self.gps_min_xy_std, math.sqrt(xy_var))

    def _gps_fix_to_map_xyz(self, fix: NavSatFix) -> Tuple[float, float, float]:
        lat0 = math.radians(self.gps_map_origin_lat)
        d_lat = math.radians(fix.latitude - self.gps_map_origin_lat)
        d_lon = math.radians(fix.longitude - self.gps_map_origin_lon)
        east = self.EARTH_RADIUS_M * d_lon * math.cos(lat0)
        north = self.EARTH_RADIUS_M * d_lat

        yaw = self.gps_map_yaw_from_enu
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x = cos_yaw * east - sin_yaw * north
        y = sin_yaw * east + cos_yaw * north
        z = 0.0
        if math.isfinite(fix.altitude):
            z = fix.altitude - self.gps_map_origin_alt
        return x, y, z

    def _sample_from_odom(self, msg: Odometry, now_sec: float) -> PoseSample:
        pose = msg.pose.pose
        return PoseSample(
            time_sec=now_sec,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            yaw=self._yaw_from_pose(pose),
            msg=msg,
        )

    def _pose_sanity_reasons(self, sample: PoseSample) -> List[str]:
        if self.last_pose_sample is None:
            return []

        previous = self.last_pose_sample
        dt = sample.time_sec - previous.time_sec
        if dt <= 0.001:
            return []

        dx = sample.x - previous.x
        dy = sample.y - previous.y
        dz = sample.z - previous.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        yaw_delta = abs(self._angle_delta(sample.yaw, previous.yaw))
        speed = distance / dt
        yaw_rate = yaw_delta / dt

        reasons: List[str] = []
        if distance > self.max_pose_jump_m:
            reasons.append(
                f"pose jumped {distance:.2f}m in {dt:.2f}s"
            )
        if speed > self.max_reasonable_speed_mps:
            reasons.append(
                f"pose-derived speed {speed:.2f}m/s exceeds "
                f"{self.max_reasonable_speed_mps:.2f}m/s"
            )
        if yaw_delta > self.max_yaw_jump_rad:
            reasons.append(
                f"yaw changed {yaw_delta:.2f}rad in {dt:.2f}s"
            )
        if yaw_rate > self.max_reasonable_yaw_rate_radps:
            reasons.append(
                f"yaw-rate {yaw_rate:.2f}rad/s exceeds "
                f"{self.max_reasonable_yaw_rate_radps:.2f}rad/s"
            )

        return reasons

    def _mark_lost(self, reasons: List[str]) -> None:
        deduped = []
        for reason in reasons:
            if reason not in deduped:
                deduped.append(reason)

        if not self.lost_active:
            self.lost_active = True
            self.loss_event_id += 1
            self.recovery_attempts_this_event = 0
            self.localization_good_since = None
            self.logged_healthy = False
            self.current_loss_reasons = deduped
            self.get_logger().warn(
                "Localization suspected lost: " + "; ".join(deduped)
            )
            return

        if deduped != self.current_loss_reasons:
            self.current_loss_reasons = deduped
            self._log_throttled(
                "warn",
                "lost_reasons_update",
                5.0,
                "Localization still suspected lost: "
                + "; ".join(deduped),
            )

    def _cloud_alive(self, now_sec: float) -> bool:
        if self.last_cloud_time is None:
            return False
        return now_sec - self.last_cloud_time <= self.cloud_stale_timeout_sec

    def _cloud_ready(self, now_sec: float) -> bool:
        return (
            self._cloud_alive(now_sec)
            and self.last_cloud_points >= self.min_cloud_points_for_relocalize
        )

    def _pose_recent(self, now_sec: float) -> bool:
        if self.last_pose_time is None:
            return False
        return now_sec - self.last_pose_time <= self.pose_stale_timeout_sec

    def _startup_pose_msg(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = self.startup_pose_x
        msg.pose.pose.position.y = self.startup_pose_y
        msg.pose.pose.position.z = self.startup_pose_z
        msg.pose.pose.orientation = self._quaternion_from_yaw(
            self.startup_pose_yaw
        )
        msg.pose.covariance = self._covariance_from_std(
            self.startup_xy_std,
            self.startup_yaw_std,
        )
        return msg

    def _pose_msg_from_sample(
        self,
        sample: PoseSample,
        xy_std: float,
        yaw_std: float,
    ) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header = sample.msg.header
        if not msg.header.frame_id:
            msg.header.frame_id = self.map_frame
        msg.pose.pose = sample.msg.pose.pose
        msg.pose.covariance = self._covariance_from_std(xy_std, yaw_std)
        return msg

    def _pose_stamped_from_cov(
        self,
        msg: PoseWithCovarianceStamped,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        return pose

    def _covariance_from_std(
        self,
        xy_std: float,
        yaw_std: float,
        z_std: Optional[float] = None,
    ) -> List[float]:
        covariance = [0.0] * 36
        xy_var = xy_std * xy_std
        yaw_var = yaw_std * yaw_std
        z_var = (z_std if z_std is not None else max(0.1, xy_std)) ** 2
        covariance[0] = xy_var
        covariance[7] = xy_var
        covariance[14] = z_var
        covariance[21] = math.pi * math.pi
        covariance[28] = math.pi * math.pi
        covariance[35] = yaw_var
        return covariance

    def _parse_diagnostics_text(self, text: str) -> Dict[str, Any]:
        fields: Dict[str, Any] = {}
        stripped = text.strip()
        if not stripped:
            return fields

        parsed = self._parse_structured_diagnostics(stripped)
        if isinstance(parsed, dict):
            self._flatten_diagnostics(parsed, fields)

        for key, value in re.findall(
            r"([A-Za-z_][\w./-]*)\s*[:=]\s*([^\s,;]+)",
            stripped,
        ):
            fields[self._normalize_key(key)] = self._normalize_value(value)

        return fields

    def _parse_structured_diagnostics(self, text: str) -> Any:
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        if yaml is None:
            return None

        try:
            return yaml.safe_load(text)
        except yaml.YAMLError:
            return None

    def _flatten_diagnostics(
        self,
        value: Any,
        out: Dict[str, Any],
        prefix: str = "",
    ) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                next_prefix = f"{prefix}.{key}" if prefix else str(key)
                self._flatten_diagnostics(child, out, next_prefix)
            return

        if prefix:
            out[self._normalize_key(prefix)] = self._normalize_value(value)

    def _diag_value(self, fields: Dict[str, Any], wanted: str) -> Any:
        wanted_norm = self._normalize_key(wanted)
        for key, value in fields.items():
            if key == wanted_norm or key.endswith("." + wanted_norm):
                return value
        return None

    def _diag_float(
        self,
        fields: Dict[str, Any],
        wanted: str,
    ) -> Optional[float]:
        value = self._diag_value(fields, wanted)
        if isinstance(value, bool):
            return 1.0 if value else 0.0
        if isinstance(value, (float, int)):
            return float(value)
        if isinstance(value, str):
            try:
                return float(value)
            except ValueError:
                return None
        return None

    def _normalize_key(self, key: str) -> str:
        return key.strip().lower().replace("-", "_").replace("/", ".")

    def _normalize_value(self, value: Any) -> Any:
        if isinstance(value, bool):
            return value
        if isinstance(value, (float, int)):
            return float(value)
        text = str(value).strip().strip("'\"")
        lowered = text.lower()
        if lowered in BOOL_TRUE:
            return True
        if lowered in BOOL_FALSE:
            return False
        try:
            return float(text)
        except ValueError:
            return text

    def _yaw_from_pose(self, pose: Pose) -> float:
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _quaternion_from_yaw(self, yaw: float):
        q = Quaternion()
        half_yaw = yaw * 0.5
        q.w = math.cos(half_yaw)
        q.z = math.sin(half_yaw)
        return q

    def _angle_delta(self, current: float, previous: float) -> float:
        return math.atan2(
            math.sin(current - previous),
            math.cos(current - previous),
        )

    def _log_throttled(
        self,
        level: str,
        key: str,
        period_sec: float,
        message: str,
    ) -> None:
        now_sec = self._now_sec()
        last = self._last_log_times.get(key)
        if last is not None and now_sec - last < period_sec:
            return
        self._last_log_times[key] = now_sec
        logger = self.get_logger()
        if level == "debug":
            logger.debug(message)
        elif level == "info":
            logger.info(message)
        elif level == "warn":
            logger.warn(message)
        else:
            logger.error(message)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MolaAutoLocalizationSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
