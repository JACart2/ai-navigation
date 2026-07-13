"""Conservative MOLA auto-localization supervisor."""

import importlib
import json
import math
import os
import re
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

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
    max_pose_covariance: Optional[float] = None


@dataclass
class GpsFixSample:
    time_sec: float
    stamp_sec: Optional[float]
    latitude: float
    longitude: float
    altitude: float
    status: int
    max_covariance: Optional[float]
    covariance_x: Optional[float] = None
    covariance_y: Optional[float] = None
    covariance_z: Optional[float] = None
    covariance_type: Optional[int] = None
    frame_id: str = ""


@dataclass
class GpsMapSeedSample:
    time_sec: float
    fix_time_sec: float
    latitude: float
    longitude: float
    map_x: float
    map_y: float
    z: float
    yaw: float
    yaw_source: str
    status: int
    max_covariance: Optional[float]
    altitude: Optional[float] = None
    covariance_x: Optional[float] = None
    covariance_y: Optional[float] = None
    covariance_z: Optional[float] = None
    covariance_type: Optional[int] = None
    frame_id: str = ""


@dataclass
class GpsYawSweepState:
    map_x: float
    map_y: float
    z: float
    seed_time_sec: float
    seed_fix_time_sec: float
    seed_source: str
    yaw_mode: str
    base_yaw_rad: float
    angles_deg: List[float]
    next_index: int = 0
    attempts: int = 0
    active_yaw_deg: Optional[float] = None
    active_yaw_rad: Optional[float] = None
    candidate_sent_time: Optional[float] = None
    candidate_pose_sequence: int = 0
    candidate_wait_logged: bool = False
    candidate_service_success: Optional[bool] = None
    candidate_service_message: str = ""
    candidate_promising_since: Optional[float] = None
    candidate_promising_reason: str = ""


class MolaAutoLocalizationSupervisor(Node):
    """Watch MOLA health and request cautious relocalization when needed."""

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
        self.pose_history: List[PoseSample] = []
        self.pose_sequence = 0
        self.last_good_pose: Optional[PoseWithCovarianceStamped] = None
        self.last_good_pose_time: Optional[float] = None
        self.last_pcl_pose_time: Optional[float] = None
        self.last_diag_time: Optional[float] = None
        self.last_diag_fields: Dict[str, Any] = {}
        self.bad_diag_since: Optional[float] = None
        self.bad_diag_reasons: List[str] = []
        self.localization_good_since: Optional[float] = None
        self.manual_pose_cooldown_until = 0.0
        self.manual_pose_lockout_until = 0.0
        self.manual_pose_pending = False
        self.manual_pose_time: Optional[float] = None
        self.manual_pose_promising_since: Optional[float] = None
        self.manual_pose_count = 0
        self.last_gps_msg_time: Optional[float] = None
        self.logged_first_gps_rx = False
        self.last_gps_fix: Optional[GpsFixSample] = None
        self.latest_gps_map_seed: Optional[GpsMapSeedSample] = None
        self.gps_to_map_calibration: Optional[Tuple[float, ...]] = None
        self.gps_to_map_config_loaded = False
        self.gps_to_map_config_path = ""
        self.gps_to_map_method = "unloaded"
        self.gps_yaw_sweep_state: Optional[GpsYawSweepState] = None
        self.gps_yaw_sweep_failed_event_id = 0
        self.post_recovery_lockout_until = 0.0
        self.post_recovery_lockout_reason = ""
        self.recovery_episode_id = 0
        self.supervisor_state = "STARTING"

        self.logged_cloud_wait = False
        self.logged_cloud_healthy = False
        self.logged_pose_wait = False
        self.logged_first_pose = False
        self.logged_healthy = False

        self.lost_active = False
        self.lost_since_sec: Optional[float] = None
        self.loss_event_id = 0
        self.current_loss_reasons: List[str] = []
        self.recovery_attempts_this_event = 0
        self.recovery_promising_since: Optional[float] = None
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
            PoseWithCovarianceStamped,
            self.manual_pose_topic,
            self._manual_pose_callback,
            10,
        )
        self.create_subscription(
            String,
            self.diagnostics_topic,
            self._diagnostics_callback,
            10,
        )
        if self.use_gps_relocalize:
            self.create_subscription(
                NavSatFix,
                self.gps_topic,
                self._gps_callback,
                qos_profile_sensor_data,
            )
        if self.enable_startup_relocalize or self.enable_recovery_relocalize:
            self._ensure_relocalize_client()
        self.create_timer(self.check_period_sec, self._tick)

        gps_state = "enabled" if self.use_gps_relocalize else "disabled"
        self.get_logger().info(
            "MOLA auto-localization supervisor started; "
            f"cloud={self.cloud_topic}, pose={self.mola_pose_topic}, "
            f"diagnostics={self.diagnostics_topic}, "
            f"manual_pose={self.manual_pose_topic}, "
            f"relocalize_service={self.relocalize_service}, "
            f"gps_relocalize={gps_state}"
        )
        self.get_logger().info(
            "GPS_TOPIC_CONFIG "
            f"gps_topic={self.gps_topic} "
            f"use_gps_relocalize={str(self.use_gps_relocalize).lower()} "
            f"gps_yaw_sweep_enabled="
            f"{str(self.gps_yaw_sweep_enabled).lower()}"
        )
        if self.use_gps_relocalize:
            self.get_logger().info(
                "GPS recovery seed enabled; listening on "
                f"{self.gps_topic} and using {self.gps_to_map_source} "
                "for GPS-to-map conversion."
            )
            if self.gps_yaw_sweep_enabled:
                self.get_logger().info(
                    "GPS yaw sweep enabled; configured "
                    f"{len(self.gps_yaw_sweep_angles_deg)} candidate(s)."
                )
        self.get_logger().info(
            "Waiting for LiDAR before requesting startup relocalization."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("cloud_topic", "/velodyne_points")
        self.declare_parameter("mola_pose_topic", "/lidar_odometry/pose")
        self.declare_parameter("pcl_pose_topic", "/pcl_pose")
        self.declare_parameter("manual_pose_topic", "/initialpose")
        self.declare_parameter("manual_pose_cooldown_sec", 30.0)
        self.declare_parameter("manual_pose_lockout_sec", 30.0)
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
        self.declare_parameter("use_gps_relocalize", False)
        self.declare_parameter("gps_topic", "/gps")
        self.declare_parameter("gps_fix_timeout_sec", 3.0)
        self.declare_parameter("gps_stale_timeout_sec", 3.0)
        self.declare_parameter("gps_seed_max_age_sec", 3.0)
        self.declare_parameter("gps_min_status", 0)
        self.declare_parameter("gps_max_covariance", 100.0)
        self.declare_parameter("gps_reject_unknown_covariance", False)
        self.declare_parameter("gps_relocalize_cooldown_sec", 8.0)
        self.declare_parameter("gps_seed_freeze_during_recovery", True)
        self.declare_parameter("gps_recovery_xy_std", 10.0)
        self.declare_parameter("gps_recovery_z_std", 5.0)
        self.declare_parameter("gps_recovery_yaw_std", 3.14)
        self.declare_parameter("gps_to_map_source", "landmark_calibration")
        self.declare_parameter("gps_calibration_config_path", "")
        self.declare_parameter("calibration_config_dir", "")
        self.declare_parameter("calibration_config_file", "")
        self.declare_parameter("gps_yaw_sweep_enabled", False)
        self.declare_parameter("gps_yaw_sweep_mode", "absolute")
        self.declare_parameter(
            "gps_yaw_candidates_deg",
            [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
        )
        self.declare_parameter(
            "gps_yaw_sweep_angles_deg",
            [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
        )
        self.declare_parameter("gps_yaw_sweep_attempt_delay_sec", 1.0)
        self.declare_parameter("yaw_candidate_settle_sec", 1.0)
        self.declare_parameter("yaw_candidate_timeout_sec", 7.0)
        self.declare_parameter("acceptance_near_seed_m", 6.0)
        self.declare_parameter("acceptance_min_fresh_pose_count", 3)
        self.declare_parameter("acceptance_require_diagnostics_ok", True)
        self.declare_parameter("acceptance_require_match_quality", False)
        self.declare_parameter("acceptance_allow_quality_fallback", False)
        self.declare_parameter("acceptance_confirmation_window_sec", 2.0)
        self.declare_parameter("acceptance_require_pose_stability", True)
        self.declare_parameter("acceptance_pose_stability_window_sec", 2.0)
        self.declare_parameter("acceptance_max_pose_drift_m", 1.0)
        self.declare_parameter("acceptance_max_yaw_drift_rad", 0.35)
        self.declare_parameter("acceptance_min_icp_quality", 0.75)
        self.declare_parameter("acceptance_min_inlier_ratio", 0.60)
        self.declare_parameter("acceptance_min_matched_points", 0)
        self.declare_parameter("acceptance_max_icp_error", 0.75)
        self.declare_parameter("acceptance_max_pose_covariance", 4.0)
        self.declare_parameter("acceptance_max_dropped_frames", 0.10)
        self.declare_parameter("acceptance_required_diagnostic_level", 0)
        self.declare_parameter("gps_yaw_sweep_accept_icp_quality", 0.0)
        self.declare_parameter(
            "gps_yaw_sweep_accept_dropped_frames_ratio",
            0.2,
        )
        self.declare_parameter("gps_yaw_sweep_max_attempts", 8)
        self.declare_parameter("gps_yaw_sweep_candidate_yaw_std", 1.57)
        self.declare_parameter("post_recovery_lockout_sec", 25.0)
        self.declare_parameter("recovery_require_sustained_loss", True)
        self.declare_parameter("recovery_min_lost_duration_sec", 3.0)
        self.declare_parameter("enable_startup_relocalize", True)
        self.declare_parameter("enable_recovery_relocalize", True)
        self.declare_parameter("max_relocalize_attempts_per_event", 3)
        self.declare_parameter("service_wait_timeout_sec", 0.05)
        self.declare_parameter("relocalize_response_timeout_sec", 10.0)
        self.declare_parameter("debug", False)

    def _read_parameters(self) -> None:
        self.cloud_topic = self._param_str("cloud_topic")
        self.mola_pose_topic = self._param_str("mola_pose_topic")
        self.pcl_pose_topic = self._param_str("pcl_pose_topic")
        self.manual_pose_topic = self._param_str("manual_pose_topic")
        self.manual_pose_cooldown_sec = max(
            0.0,
            self._param_float("manual_pose_cooldown_sec"),
        )
        self.manual_pose_lockout_sec = max(
            0.0,
            self._param_float("manual_pose_lockout_sec"),
        )
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
        self.use_gps_relocalize = self._param_bool("use_gps_relocalize")
        self.gps_topic = self._param_str("gps_topic")
        self.gps_fix_timeout_sec = max(
            0.0,
            self._param_float("gps_fix_timeout_sec"),
        )
        self.gps_stale_timeout_sec = max(
            0.0,
            self._param_float("gps_stale_timeout_sec"),
        )
        self.gps_seed_max_age_sec = max(
            0.0,
            self._param_float("gps_seed_max_age_sec"),
        )
        self.gps_min_status = self._param_int("gps_min_status")
        self.gps_max_covariance = max(
            0.0,
            self._param_float("gps_max_covariance"),
        )
        self.gps_reject_unknown_covariance = self._param_bool(
            "gps_reject_unknown_covariance"
        )
        self.gps_relocalize_cooldown_sec = max(
            0.0,
            self._param_float("gps_relocalize_cooldown_sec"),
        )
        if self.gps_relocalize_cooldown_sec == 0.0:
            self.gps_relocalize_cooldown_sec = self.relocalize_cooldown_sec
        self.gps_seed_freeze_during_recovery = self._param_bool(
            "gps_seed_freeze_during_recovery"
        )
        self.gps_recovery_xy_std = max(
            0.1,
            self._param_float("gps_recovery_xy_std"),
        )
        self.gps_recovery_z_std = max(
            0.1,
            self._param_float("gps_recovery_z_std"),
        )
        self.gps_recovery_yaw_std = max(
            0.1,
            self._param_float("gps_recovery_yaw_std"),
        )
        self.gps_to_map_source = (
            self._param_str("gps_to_map_source").strip().lower()
        )
        self.gps_calibration_config_path = self._param_str(
            "gps_calibration_config_path"
        ).strip()
        self.calibration_config_dir = self._param_str(
            "calibration_config_dir"
        ).strip()
        self.calibration_config_file = self._param_str(
            "calibration_config_file"
        ).strip()
        self.gps_yaw_sweep_enabled = self._param_bool(
            "gps_yaw_sweep_enabled"
        )
        self.gps_yaw_sweep_mode = (
            self._param_str("gps_yaw_sweep_mode").strip().lower()
        )
        if self.gps_yaw_sweep_mode not in ("absolute", "relative"):
            self.get_logger().warn(
                "gps_yaw_sweep_mode must be 'absolute' or 'relative'; "
                "using absolute."
            )
            self.gps_yaw_sweep_mode = "absolute"
        self.gps_yaw_sweep_angles_deg = self._gps_yaw_sweep_angles_from_param()
        legacy_settle_sec = max(
            0.1,
            self._param_float("gps_yaw_sweep_attempt_delay_sec"),
        )
        self.yaw_candidate_settle_sec = max(
            0.1,
            self._param_float("yaw_candidate_settle_sec"),
        )
        if self.yaw_candidate_settle_sec == 1.0 and legacy_settle_sec != 1.0:
            self.yaw_candidate_settle_sec = legacy_settle_sec
        self.gps_yaw_sweep_attempt_delay_sec = self.yaw_candidate_settle_sec
        self.yaw_candidate_timeout_sec = max(
            self.yaw_candidate_settle_sec,
            self._param_float("yaw_candidate_timeout_sec"),
        )
        self.acceptance_near_seed_m = max(
            0.1,
            self._param_float("acceptance_near_seed_m"),
        )
        self.acceptance_min_fresh_pose_count = max(
            1,
            self._param_int("acceptance_min_fresh_pose_count"),
        )
        self.acceptance_require_diagnostics_ok = self._param_bool(
            "acceptance_require_diagnostics_ok"
        )
        self.acceptance_require_match_quality = self._param_bool(
            "acceptance_require_match_quality"
        )
        self.acceptance_allow_quality_fallback = self._param_bool(
            "acceptance_allow_quality_fallback"
        )
        self.acceptance_confirmation_window_sec = max(
            0.0,
            self._param_float("acceptance_confirmation_window_sec"),
        )
        self.acceptance_require_pose_stability = self._param_bool(
            "acceptance_require_pose_stability"
        )
        self.acceptance_pose_stability_window_sec = max(
            0.0,
            self._param_float("acceptance_pose_stability_window_sec"),
        )
        self.acceptance_max_pose_drift_m = max(
            0.0,
            self._param_float("acceptance_max_pose_drift_m"),
        )
        self.acceptance_max_yaw_drift_rad = max(
            0.0,
            self._param_float("acceptance_max_yaw_drift_rad"),
        )
        self.acceptance_min_icp_quality = self._param_float(
            "acceptance_min_icp_quality"
        )
        self.acceptance_min_inlier_ratio = self._param_float(
            "acceptance_min_inlier_ratio"
        )
        self.acceptance_min_matched_points = max(
            0,
            self._param_int("acceptance_min_matched_points"),
        )
        self.acceptance_max_icp_error = self._param_float(
            "acceptance_max_icp_error"
        )
        self.acceptance_max_pose_covariance = max(
            0.0,
            self._param_float("acceptance_max_pose_covariance"),
        )
        self.acceptance_max_dropped_frames = self._param_float(
            "acceptance_max_dropped_frames"
        )
        self.acceptance_required_diagnostic_level = self._param_int(
            "acceptance_required_diagnostic_level"
        )
        self.gps_yaw_sweep_accept_icp_quality = self._param_float(
            "gps_yaw_sweep_accept_icp_quality"
        )
        self.gps_yaw_sweep_accept_dropped_frames_ratio = self._param_float(
            "gps_yaw_sweep_accept_dropped_frames_ratio"
        )
        self.gps_yaw_sweep_max_attempts = max(
            1,
            self._param_int("gps_yaw_sweep_max_attempts"),
        )
        self.gps_yaw_sweep_candidate_yaw_std = max(
            0.1,
            self._param_float("gps_yaw_sweep_candidate_yaw_std"),
        )
        self.post_recovery_lockout_sec = max(
            0.0,
            self._param_float("post_recovery_lockout_sec"),
        )
        self.recovery_require_sustained_loss = self._param_bool(
            "recovery_require_sustained_loss"
        )
        self.recovery_min_lost_duration_sec = max(
            0.0,
            self._param_float("recovery_min_lost_duration_sec"),
        )
        self.enable_startup_relocalize = self._param_bool(
            "enable_startup_relocalize"
        )
        self.enable_recovery_relocalize = self._param_bool(
            "enable_recovery_relocalize"
        )
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

    def _gps_yaw_sweep_angles_from_param(self) -> List[float]:
        default_angles = [
            0.0,
            45.0,
            90.0,
            135.0,
            180.0,
            225.0,
            270.0,
            315.0,
        ]
        legacy = self._gps_yaw_angles_from_value(
            self.get_parameter("gps_yaw_sweep_angles_deg").value,
            "gps_yaw_sweep_angles_deg",
        )
        canonical = self._gps_yaw_angles_from_value(
            self.get_parameter("gps_yaw_candidates_deg").value,
            "gps_yaw_candidates_deg",
        )

        if (
            canonical
            and canonical != default_angles
            and legacy == default_angles
        ):
            return canonical
        if legacy:
            return legacy
        if canonical:
            return canonical

        self.get_logger().warn(
            "GPS yaw candidate list is empty or invalid; using "
            "default 8-heading sweep."
        )
        return default_angles

    def _gps_yaw_angles_from_value(
        self,
        value: Any,
        param_name: str,
    ) -> List[float]:
        raw_values: List[Any]
        if isinstance(value, (list, tuple)):
            raw_values = list(value)
        else:
            text = str(value).strip().strip("[]")
            raw_values = [
                item.strip()
                for item in text.split(",")
                if item.strip()
            ]

        angles: List[float] = []
        for item in raw_values:
            try:
                angle = float(item) % 360.0
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f"Ignoring invalid {param_name} angle '{item}'."
                )
                continue
            if not any(abs(angle - existing) < 1e-6 for existing in angles):
                angles.append(angle)

        return angles

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
        self.pose_history.append(sample)
        self._prune_pose_history(now_sec)
        self.pose_sequence += 1

        if not self.logged_first_pose:
            self.logged_first_pose = True
            self.get_logger().info("Received first MOLA pose.")

        if reasons:
            self.localization_good_since = None
            self.logged_healthy = False
            self._mark_lost(reasons, now_sec)
            return

        # Do not immediately trust this pose as a recovery anchor.
        # The first MOLA pose can be a bad startup guess. A pose is only promoted
        # to last_good_pose by _update_healthy_state() after it has remained
        # recent, sane, and diagnostically healthy for min_good_pose_age_sec.

    def _pcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        del msg
        self.last_pcl_pose_time = self._now_sec()
        self._log_throttled(
            "info",
            "first_pcl_pose",
            3600.0,
            "Observed /pcl_pose relay; using it as secondary visibility only.",
        )

    def _manual_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        del msg
        now_sec = self._now_sec()
        self.manual_pose_count += 1
        self.manual_pose_cooldown_until = (
            now_sec + self.manual_pose_cooldown_sec
        )
        self.manual_pose_pending = True
        self.manual_pose_time = now_sec
        self.manual_pose_promising_since = None
        cancelled_sweep = self._cancel_gps_yaw_sweep(
            "Manual pose received; cancelling GPS yaw sweep / suppressing "
            "recovery during cooldown."
        )
        if not cancelled_sweep:
            self.get_logger().warn(
                "Manual pose received; cancelling GPS yaw sweep / "
                "suppressing recovery during cooldown."
            )

        # A human/RViz correction should become the new source of truth.
        # Clear the old recovery anchor so the supervisor cannot drag MOLA
        # back toward a stale startup pose.
        self.last_good_pose = None
        self.last_good_pose_time = None
        self.localization_good_since = None
        self.logged_healthy = False
        self.recovery_promising_since = None
        self.lost_active = False
        self.lost_since_sec = None
        self.current_loss_reasons = []
        self.recovery_attempts_this_event = 0

        # A manual pose supersedes the automatic broad startup guess.
        # Otherwise startup relocalization can fire later and fight the user.
        self.startup_relocalize_complete = True

        self.get_logger().warn(
            "Manual initial pose received on "
            f"{self.manual_pose_topic}; cleared last_good_pose and pausing "
            f"auto-relocalization for {self.manual_pose_cooldown_sec:.1f}s. "
            "If MOLA becomes healthy from this manual seed, it will be "
            "accepted and held with a manual lockout."
        )

    def _manual_pose_cooldown_active(self, now_sec: float) -> bool:
        return self.manual_pose_cooldown_until > now_sec

    def _manual_pose_lockout_active(self, now_sec: float) -> bool:
        return self.manual_pose_lockout_until > now_sec

    def _post_recovery_lockout_active(self, now_sec: float) -> bool:
        return self.post_recovery_lockout_until > now_sec

    def _auto_relocalize_lockout_active(self, now_sec: float) -> bool:
        return (
            self._manual_pose_cooldown_active(now_sec)
            or self._manual_pose_lockout_active(now_sec)
            or self._post_recovery_lockout_active(now_sec)
        )

    def _gps_callback(self, msg: NavSatFix) -> None:
        now_sec = self._now_sec()
        self.last_gps_msg_time = now_sec
        if not self.logged_first_gps_rx:
            self.logged_first_gps_rx = True
            self.get_logger().info(
                "GPS_RX "
                f"topic={self.gps_topic} "
                f"lat={self._format_optional_float(self._safe_float(getattr(msg, 'latitude', None)), 12)} "
                f"lon={self._format_optional_float(self._safe_float(getattr(msg, 'longitude', None)), 12)} "
                f"status={self._format_optional_int(self._gps_fix_status_value(msg))} "
                f"cov_x={self._format_optional_float(self._gps_fix_covariance_value(msg, 0), 6)} "
                f"cov_y={self._format_optional_float(self._gps_fix_covariance_value(msg, 4), 6)}"
            )
        reject_reason = self._gps_fix_reject_reason(msg, now_sec)
        if reject_reason:
            self._log_gps_conversion_from_msg(
                msg,
                accepted=False,
                reason=f"fix_rejected:{reject_reason}",
                level="info",
            )
            self._log_throttled(
                "debug",
                "gps_fix_rejected",
                5.0,
                f"GPS fix ignored: {reject_reason}.",
            )
            return

        self.last_gps_fix = GpsFixSample(
            time_sec=now_sec,
            stamp_sec=self._gps_msg_stamp_sec(msg),
            latitude=float(msg.latitude),
            longitude=float(msg.longitude),
            altitude=float(msg.altitude),
            status=int(msg.status.status),
            max_covariance=self._gps_fix_max_covariance(msg),
            covariance_x=self._gps_fix_covariance_value(msg, 0),
            covariance_y=self._gps_fix_covariance_value(msg, 4),
            covariance_z=self._gps_fix_covariance_value(msg, 8),
            covariance_type=self._gps_fix_covariance_type(msg),
            frame_id=str(getattr(getattr(msg, "header", None), "frame_id", "")),
        )
        self._log_throttled(
            "info",
            "gps_fix_valid",
            60.0,
            "Received valid GPS fix for recovery relocalization "
            f"(status={self.last_gps_fix.status}).",
        )
        self._update_latest_gps_map_seed(self.last_gps_fix)

    def _gps_fix_reject_reason(
        self,
        msg: NavSatFix,
        now_sec: Optional[float] = None,
    ) -> str:
        if now_sec is None:
            now_sec = self._now_sec()

        try:
            status = int(msg.status.status)
        except (AttributeError, TypeError, ValueError):
            return "missing NavSatStatus"

        if status < self.gps_min_status:
            return f"status {status} below minimum {self.gps_min_status}"
        if not math.isfinite(msg.latitude):
            return "latitude is not finite"
        if not math.isfinite(msg.longitude):
            return "longitude is not finite"
        if msg.latitude < -90.0 or msg.latitude > 90.0:
            return f"latitude {msg.latitude:.8f} outside valid range"
        if msg.longitude < -180.0 or msg.longitude > 180.0:
            return f"longitude {msg.longitude:.8f} outside valid range"

        stamp_sec = self._gps_msg_stamp_sec(msg)
        if stamp_sec is not None and self.gps_fix_timeout_sec > 0.0:
            stamp_age = now_sec - stamp_sec
            if stamp_age > self.gps_fix_timeout_sec:
                return (
                    f"message stamp stale for {stamp_age:.1f}s "
                    f"(timeout {self.gps_fix_timeout_sec:.1f}s)"
                )
            if stamp_age < -self.gps_fix_timeout_sec:
                return (
                    f"message stamp is {-stamp_age:.1f}s in the future"
                )

        covariance_reason = self._gps_covariance_reject_reason(msg)
        if covariance_reason:
            return covariance_reason

        return ""

    def _gps_msg_stamp_sec(self, msg: NavSatFix) -> Optional[float]:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return None

        try:
            sec = float(stamp.sec)
            nanosec = float(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return None

        stamp_sec = sec + nanosec * 1e-9
        if stamp_sec <= 0.0:
            return None
        return stamp_sec

    def _gps_covariance_reject_reason(self, msg: NavSatFix) -> str:
        try:
            covariance_type = int(
                getattr(msg, "position_covariance_type", 0)
            )
        except (TypeError, ValueError):
            covariance_type = 0
        unknown_type = getattr(NavSatFix, "COVARIANCE_TYPE_UNKNOWN", 0)
        if (
            covariance_type == unknown_type
            and self.gps_reject_unknown_covariance
        ):
            return "position covariance type is unknown"

        covariance = getattr(msg, "position_covariance", None)
        if covariance is None or len(covariance) < 9:
            return ""

        diagonal = []
        for index in (0, 4, 8):
            try:
                value = float(covariance[index])
            except (TypeError, ValueError):
                return "position covariance contains non-numeric values"
            if not math.isfinite(value):
                return "position covariance contains non-finite values"
            if value < 0.0:
                return "position covariance contains negative variances"
            diagonal.append(value)

        if covariance_type == unknown_type and max(diagonal) == 0.0:
            return ""

        if (
            self.gps_max_covariance > 0.0
            and max(diagonal) > self.gps_max_covariance
        ):
            return (
                "position covariance "
                f"{max(diagonal):.3f} exceeds "
                f"{self.gps_max_covariance:.3f}"
            )
        return ""

    def _gps_fix_covariance_type(self, msg: NavSatFix) -> Optional[int]:
        try:
            return int(getattr(msg, "position_covariance_type", 0))
        except (TypeError, ValueError):
            return None

    def _gps_fix_covariance_value(
        self,
        msg: NavSatFix,
        index: int,
    ) -> Optional[float]:
        covariance = getattr(msg, "position_covariance", None)
        if covariance is None or len(covariance) <= index:
            return None
        try:
            value = float(covariance[index])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return value

    def _gps_fix_max_covariance(self, msg: NavSatFix) -> Optional[float]:
        covariance = getattr(msg, "position_covariance", None)
        if covariance is None or len(covariance) < 9:
            return None

        values = []
        for index in (0, 4, 8):
            try:
                value = float(covariance[index])
            except (TypeError, ValueError):
                return None
            if not math.isfinite(value):
                return None
            values.append(value)

        if not values:
            return None
        return max(values)

    def _log_gps_conversion_from_msg(
        self,
        msg: NavSatFix,
        accepted: bool,
        reason: str,
        level: str,
    ) -> None:
        self._emit_gps_conversion_log(
            latitude=self._safe_float(getattr(msg, "latitude", None)),
            longitude=self._safe_float(getattr(msg, "longitude", None)),
            altitude=self._safe_float(getattr(msg, "altitude", None)),
            status=self._gps_fix_status_value(msg),
            covariance_x=self._gps_fix_covariance_value(msg, 0),
            covariance_y=self._gps_fix_covariance_value(msg, 4),
            covariance_z=self._gps_fix_covariance_value(msg, 8),
            covariance_type=self._gps_fix_covariance_type(msg),
            frame_id=str(getattr(getattr(msg, "header", None), "frame_id", "")),
            map_x=None,
            map_y=None,
            accepted=accepted,
            reason=reason,
            level=level,
        )

    def _log_gps_conversion(
        self,
        fix: GpsFixSample,
        map_x: Optional[float],
        map_y: Optional[float],
        accepted: bool,
        reason: str,
        level: str,
    ) -> None:
        self._emit_gps_conversion_log(
            latitude=fix.latitude,
            longitude=fix.longitude,
            altitude=fix.altitude,
            status=fix.status,
            covariance_x=fix.covariance_x,
            covariance_y=fix.covariance_y,
            covariance_z=fix.covariance_z,
            covariance_type=fix.covariance_type,
            frame_id=fix.frame_id,
            map_x=map_x,
            map_y=map_y,
            accepted=accepted,
            reason=reason,
            level=level,
        )

    def _log_gps_seed_conversion(
        self,
        seed: GpsMapSeedSample,
        reason: str,
        level: str = "info",
    ) -> None:
        self._emit_gps_conversion_log(
            latitude=seed.latitude,
            longitude=seed.longitude,
            altitude=seed.altitude,
            status=seed.status,
            covariance_x=seed.covariance_x,
            covariance_y=seed.covariance_y,
            covariance_z=seed.covariance_z,
            covariance_type=seed.covariance_type,
            frame_id=seed.frame_id,
            map_x=seed.map_x,
            map_y=seed.map_y,
            accepted=True,
            reason=reason,
            level=level,
        )

    def _emit_gps_conversion_log(
        self,
        latitude: Optional[float],
        longitude: Optional[float],
        altitude: Optional[float],
        status: Optional[int],
        covariance_x: Optional[float],
        covariance_y: Optional[float],
        covariance_z: Optional[float],
        covariance_type: Optional[int],
        frame_id: str,
        map_x: Optional[float],
        map_y: Optional[float],
        accepted: bool,
        reason: str,
        level: str,
    ) -> None:
        message = (
            "GPS_CONVERSION "
            f"topic={self._log_token(getattr(self, 'gps_topic', '/gps'))} "
            f"frame_id={self._log_token(frame_id)} "
            f"lat={self._format_optional_float(latitude, 12)} "
            f"lon={self._format_optional_float(longitude, 12)} "
            f"alt={self._format_optional_float(altitude, 3)} "
            f"status={self._format_optional_int(status)} "
            f"cov_x={self._format_optional_float(covariance_x, 6)} "
            f"cov_y={self._format_optional_float(covariance_y, 6)} "
            f"cov_z={self._format_optional_float(covariance_z, 6)} "
            f"cov_type={self._format_optional_int(covariance_type)} "
            f"-> map_x={self._format_optional_float(map_x, 3)} "
            f"map_y={self._format_optional_float(map_y, 3)} "
            f"accepted={str(accepted).lower()} "
            f"reason={self._log_token(reason)} "
            f"{self._gps_conversion_context_text()}"
        )

        logger = self.get_logger()
        if level == "debug":
            logger.debug(message)
        elif level == "warn":
            logger.warn(message)
        elif level == "error":
            logger.error(message)
        else:
            logger.info(message)

    def _gps_fix_status_value(self, msg: NavSatFix) -> Optional[int]:
        try:
            return int(getattr(getattr(msg, "status", None), "status", None))
        except (TypeError, ValueError):
            return None

    def _safe_float(self, value: Any) -> Optional[float]:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _format_optional_float(
        self,
        value: Optional[float],
        precision: int,
    ) -> str:
        if value is None:
            return "none"
        try:
            number = float(value)
        except (TypeError, ValueError):
            return "none"
        if not math.isfinite(number):
            return str(number).lower()
        return f"{number:.{precision}f}"

    def _format_optional_int(self, value: Optional[int]) -> str:
        if value is None:
            return "none"
        try:
            return str(int(value))
        except (TypeError, ValueError):
            return "none"

    def _log_token(self, value: Any) -> str:
        text = str(value).strip()
        if not text:
            return "none"
        return re.sub(r"\s+", "_", text)

    def _gps_conversion_context_text(self) -> str:
        source = self._canonical_gps_to_map_source()
        method = self._log_token(
            getattr(self, "gps_to_map_method", "unloaded")
        )
        config_path = self._gps_conversion_config_path_for_log()
        map_frame = self._log_token(getattr(self, "map_frame", "map"))
        text = (
            f"method={method} source={source} "
            f"config={self._log_token(config_path)} "
            f"map_frame={map_frame} units=m scale=1.0 utm_zone=none"
        )

        calibration = getattr(self, "gps_to_map_calibration", None)
        if calibration is None or len(calibration) < 7:
            return text + " calibration=unloaded"

        (
            ref_lat,
            ref_lon,
            cx_local,
            cy_local,
            cx_gps,
            cy_gps,
            theta_degrees,
        ) = calibration[:7]
        meters_per_degree_lat = self._meters_per_degree_lat(ref_lat)
        meters_per_degree_lon = self._meters_per_degree_lon(ref_lat)
        return (
            text
            + f" origin_lat={ref_lat:.12f} origin_lon={ref_lon:.12f} "
            f"cx_local={cx_local:.6f} cy_local={cy_local:.6f} "
            f"cx_gps={cx_gps:.6f} cy_gps={cy_gps:.6f} "
            f"rotation_deg={theta_degrees:.9f} "
            f"meters_per_degree_lat={meters_per_degree_lat:.6f} "
            f"meters_per_degree_lon={meters_per_degree_lon:.6f}"
        )

    def _canonical_gps_to_map_source(self) -> str:
        source = str(getattr(self, "gps_to_map_source", "")).strip().lower()
        if source in ("", "landmark", "landmarks"):
            return "landmark_calibration"
        return self._log_token(source)

    def _gps_conversion_config_path_for_log(self) -> str:
        loaded_path = getattr(self, "gps_to_map_config_path", "")
        if loaded_path:
            return loaded_path

        explicit_path = getattr(self, "gps_calibration_config_path", "")
        if explicit_path:
            return os.path.abspath(os.path.expanduser(explicit_path))

        config_dir = getattr(self, "calibration_config_dir", "")
        config_file = getattr(self, "calibration_config_file", "")
        if config_dir and config_file:
            return os.path.abspath(
                os.path.join(os.path.expanduser(config_dir), config_file)
            )

        return ""

    def _update_latest_gps_map_seed(self, fix: GpsFixSample) -> bool:
        try:
            map_x, map_y = self._gps_to_map_xy(fix)
        except ValueError as exc:
            self._log_gps_conversion(
                fix,
                map_x=None,
                map_y=None,
                accepted=False,
                reason=f"conversion_failed:{exc}",
                level="warn",
            )
            self._log_throttled(
                "warn",
                "gps_seed_conversion_failed",
                10.0,
                "Valid GPS fix cannot be used as a map seed yet: "
                f"{exc}.",
            )
            return False

        if self.last_good_pose is not None:
            yaw_source = "last_good_pose"
        else:
            yaw_source = "startup_pose"

        self.latest_gps_map_seed = GpsMapSeedSample(
            time_sec=self._now_sec(),
            fix_time_sec=fix.time_sec,
            latitude=fix.latitude,
            longitude=fix.longitude,
            map_x=map_x,
            map_y=map_y,
            z=self._gps_seed_z(),
            yaw=self._gps_seed_yaw(),
            yaw_source=yaw_source,
            status=fix.status,
            max_covariance=fix.max_covariance,
            altitude=fix.altitude,
            covariance_x=fix.covariance_x,
            covariance_y=fix.covariance_y,
            covariance_z=fix.covariance_z,
            covariance_type=fix.covariance_type,
            frame_id=fix.frame_id,
        )
        if self.gps_yaw_sweep_state is None:
            seed_policy = "latest_seed_updated"
        elif self.gps_seed_freeze_during_recovery:
            seed_policy = "latest_seed_updated_active_yaw_seed_frozen"
        else:
            seed_policy = "latest_seed_updated_active_yaw_seed_movable"
        self._log_gps_conversion(
            fix,
            map_x=map_x,
            map_y=map_y,
            accepted=True,
            reason=seed_policy,
            level="info",
        )
        covariance_text = self._metric_text(fix.max_covariance)
        self._log_throttled(
            "info",
            "gps_map_seed_updated",
            10.0,
            "Updated GPS-derived map seed for relocalization only: "
            f"x={map_x:.2f}, y={map_y:.2f}, "
            f"yaw_source={yaw_source}, max_covariance={covariance_text}. "
            "GPS is only a seed; MOLA/LiDAR remains the localization "
            "authority.",
        )
        if self.gps_yaw_sweep_state is not None:
            if self.gps_seed_freeze_during_recovery:
                self._log_throttled(
                    "debug",
                    "gps_seed_frozen_during_recovery",
                    5.0,
                    "GPS update received during active recovery episode; "
                    "active yaw sweep keeps its frozen x/y seed.",
                )
            elif self.gps_yaw_sweep_state.candidate_sent_time is None:
                self.gps_yaw_sweep_state.map_x = map_x
                self.gps_yaw_sweep_state.map_y = map_y
                self.gps_yaw_sweep_state.z = self._gps_seed_z()
                self.gps_yaw_sweep_state.seed_time_sec = self._now_sec()
                self.gps_yaw_sweep_state.seed_fix_time_sec = fix.time_sec
                self.get_logger().warn(
                    "GPS update moved active recovery seed because "
                    "gps_seed_freeze_during_recovery is false: "
                    f"x={map_x:.2f}, y={map_y:.2f}."
                )
        return True

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
            self._mark_lost(reasons, now_sec)
        else:
            self._update_healthy_state(now_sec)

        self._update_supervisor_state(now_sec)
        self._cancel_recovery_activity_if_healthy(now_sec)

        if self.gps_yaw_sweep_state is not None:
            self._process_gps_yaw_sweep(now_sec)
            return

        self._maybe_startup_relocalize(now_sec)
        self._maybe_recovery_relocalize(now_sec)

    def _update_supervisor_state(self, now_sec: float) -> None:
        if self.pending_relocalize_future is not None:
            state = "RECOVERY_PENDING"
        elif self._manual_pose_cooldown_active(now_sec):
            state = "MANUAL_POSE_COOLDOWN"
        elif self._manual_pose_lockout_active(now_sec):
            state = "MANUAL_POSE_LOCKOUT"
        elif self._post_recovery_lockout_active(now_sec):
            state = "POST_RECOVERY_LOCKOUT"
        elif self.lost_active:
            state = "DEGRADED_LOST"
            if self.last_relocalize_time is not None:
                cooldown = self._active_relocalize_cooldown_sec()
                if now_sec - self.last_relocalize_time < cooldown:
                    state = "FAILED_COOLDOWN"
        elif self._pose_recent(now_sec) and self.bad_diag_since is None:
            state = "HEALTHY"
        else:
            state = "STARTING"

        if state == self.supervisor_state:
            return

        self.supervisor_state = state
        self.get_logger().info(
            "MOLA auto-localization supervisor state: "
            f"{self.supervisor_state}."
        )

    def _log_waiting_states(self, now_sec: float) -> None:
        if self.use_gps_relocalize and self.last_gps_msg_time is None:
            self._log_throttled(
                "info",
                "gps_waiting_no_messages",
                10.0,
                f"GPS_WAITING topic={self.gps_topic} "
                "no messages received yet",
            )

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

    def _diagnostics_health_result(
        self,
        now_sec: float,
        since_sec: Optional[float],
        require_diagnostics: bool,
    ) -> Tuple[str, str, Optional[float], Optional[float]]:
        icp_quality = self._diag_float(self.last_diag_fields, "icp_quality")
        dropped_ratio = self._diag_float(
            self.last_diag_fields,
            "dropped_frames_ratio",
        )
        fresh_diag = (
            self.last_diag_time is not None
            and (since_sec is None or self.last_diag_time >= since_sec)
            and now_sec - self.last_diag_time <= self.pose_stale_timeout_sec
        )

        if not fresh_diag:
            if require_diagnostics:
                if self.last_diag_time is None:
                    return (
                        "pending",
                        "diagnostics unavailable",
                        icp_quality,
                        dropped_ratio,
                    )
                return (
                    "pending",
                    "diagnostics stale or not updated after candidate",
                    icp_quality,
                    dropped_ratio,
                )
            return (
                "ok",
                "diagnostics unavailable; explicit fallback mode",
                icp_quality,
                dropped_ratio,
            )

        reasons: List[str] = []
        active = self._diag_value(self.last_diag_fields, "active")
        if active is False or active == 0.0:
            reasons.append("diagnostics report inactive")

        if (
            icp_quality is not None
            and icp_quality < self.gps_yaw_sweep_accept_icp_quality
        ):
            reasons.append(
                "ICP quality "
                f"{icp_quality:.3f} below "
                f"{self.gps_yaw_sweep_accept_icp_quality:.3f}"
            )

        if (
            dropped_ratio is not None
            and dropped_ratio
            > self.gps_yaw_sweep_accept_dropped_frames_ratio
        ):
            reasons.append(
                "dropped frame ratio "
                f"{dropped_ratio:.3f} above "
                f"{self.gps_yaw_sweep_accept_dropped_frames_ratio:.3f}"
            )

        if self.bad_diag_since is not None:
            reasons.append("diagnostics currently marked bad")

        if reasons:
            return "bad", "; ".join(reasons), icp_quality, dropped_ratio
        return "ok", "diagnostics=OK", icp_quality, dropped_ratio

    def _acceptance_evidence_result(
        self,
        now_sec: float,
        since_sec: float,
        fresh_pose_count: int,
        seed_xy: Optional[Tuple[float, float]],
    ) -> Tuple[str, str, Optional[float], Optional[float]]:
        icp_quality = self._diag_first_float(
            self.last_diag_fields,
            [
                "match_quality",
                "localization_quality",
                "pose_quality",
                "icp_quality",
                "icp_goodness",
                "goodness",
                "quality",
                "fitness",
            ],
        )
        dropped_ratio = self._diag_first_float(
            self.last_diag_fields,
            [
                "dropped_frames_ratio",
                "dropped_frame_ratio",
                "dropped_ratio",
                "drop_ratio",
                "dropped_frames",
            ],
        )
        reasons: List[str] = []
        pending: List[str] = []

        if fresh_pose_count < self.acceptance_min_fresh_pose_count:
            pending.append(
                "fresh_pose_count="
                f"{fresh_pose_count}/"
                f"{self.acceptance_min_fresh_pose_count}"
            )

        if self.last_pose_sample is None:
            pending.append("no MOLA pose sample available")
        elif self.last_pose_time is None:
            pending.append("no MOLA pose timestamp available")
        elif self.last_pose_time < since_sec:
            pending.append("no MOLA pose after candidate")
        elif now_sec - self.last_pose_time > self.pose_stale_timeout_sec:
            pending.append(
                f"MOLA pose stale for {now_sec - self.last_pose_time:.1f}s"
            )

        distance_text = "distance_to_seed=not_applicable"
        if seed_xy is not None and self.last_pose_sample is not None:
            dx = self.last_pose_sample.x - seed_xy[0]
            dy = self.last_pose_sample.y - seed_xy[1]
            distance_from_seed = math.hypot(dx, dy)
            distance_text = f"distance_to_seed={distance_from_seed:.2f}m"
            if distance_from_seed > self.acceptance_near_seed_m:
                reasons.append(
                    "distance_to_seed "
                    f"{distance_from_seed:.2f}m exceeds "
                    f"{self.acceptance_near_seed_m:.2f}m"
                )

        diag_status, diag_reason, _diag_icp, _diag_dropped = (
            self._diagnostics_health_result(
                now_sec,
                since_sec,
                self.acceptance_require_diagnostics_ok,
            )
        )
        if diag_status == "bad":
            reasons.append("diagnostics unhealthy: " + diag_reason)
        elif diag_status == "pending":
            pending.append("diagnostics not confirmed: " + diag_reason)

        match_status, match_reason, icp_quality, dropped_ratio = (
            self._match_quality_result(now_sec, since_sec)
        )
        if match_status == "bad":
            reasons.append(match_reason)
        elif match_status == "pending":
            pending.append(match_reason)

        stability_status, stability_reason = self._pose_stability_result(
            now_sec,
            since_sec,
        )
        if stability_status == "bad":
            reasons.append(stability_reason)
        elif stability_status == "pending":
            pending.append(stability_reason)

        covariance_status, covariance_reason = self._pose_covariance_result()
        if covariance_status == "bad":
            reasons.append(covariance_reason)
        elif covariance_status == "pending":
            pending.append(covariance_reason)

        detail = (
            f"{distance_text}; diagnostics={diag_reason}; "
            f"match_quality={match_reason}; "
            f"pose_stability={stability_reason}; "
            f"pose_covariance={covariance_reason}"
        )

        if reasons:
            return (
                "bad",
                "CANDIDATE_REJECTED: "
                + "; ".join(reasons)
                + "; "
                + detail,
                icp_quality,
                dropped_ratio,
            )
        if pending:
            return (
                "pending",
                "CANDIDATE_PROMISING: "
                + "; ".join(pending)
                + "; "
                + detail,
                icp_quality,
                dropped_ratio,
            )
        return (
            "ok",
            "CANDIDATE_PROMISING: all gates currently pass; " + detail,
            icp_quality,
            dropped_ratio,
        )

    def _match_quality_result(
        self,
        now_sec: float,
        since_sec: float,
    ) -> Tuple[str, str, Optional[float], Optional[float]]:
        icp_quality = self._diag_first_float(
            self.last_diag_fields,
            [
                "match_quality",
                "localization_quality",
                "pose_quality",
                "icp_quality",
                "icp_goodness",
                "goodness",
                "quality",
                "fitness",
            ],
        )
        inlier_ratio = self._diag_first_float(
            self.last_diag_fields,
            [
                "inlier_ratio",
                "inliers_ratio",
                "icp_inlier_ratio",
                "match_inlier_ratio",
                "matched_ratio",
                "inlier_fraction",
            ],
        )
        matched_points = self._diag_first_float(
            self.last_diag_fields,
            [
                "matched_points",
                "num_matched_points",
                "n_matched_points",
                "icp_matched_points",
                "inliers",
                "num_inliers",
                "matched",
            ],
        )
        icp_error = self._diag_first_float(
            self.last_diag_fields,
            [
                "icp_error",
                "icp_rmse",
                "rmse",
                "alignment_error",
                "residual_error",
                "mean_error",
                "fitness_score",
                "score",
            ],
        )
        dropped_ratio = self._diag_first_float(
            self.last_diag_fields,
            [
                "dropped_frames_ratio",
                "dropped_frame_ratio",
                "dropped_ratio",
                "drop_ratio",
                "dropped_frames",
            ],
        )
        diag_level = self._diagnostic_level_value(self.last_diag_fields)

        metric_text = (
            f"icp_quality={self._metric_text(icp_quality)}, "
            f"inlier_ratio={self._metric_text(inlier_ratio)}, "
            f"matched_points={self._metric_text(matched_points)}, "
            f"icp_error={self._metric_text(icp_error)}, "
            f"dropped_frames={self._metric_text(dropped_ratio)}, "
            f"diagnostic_level={self._metric_text(diag_level)}"
        )

        if not self.acceptance_require_match_quality:
            return (
                "ok",
                "match-quality gate disabled; " + metric_text,
                icp_quality,
                dropped_ratio,
            )

        fresh_diag = (
            self.last_diag_time is not None
            and self.last_diag_time >= since_sec
            and now_sec - self.last_diag_time <= self.pose_stale_timeout_sec
        )
        if not fresh_diag:
            return (
                "pending",
                "match-quality diagnostics unavailable or stale; "
                + metric_text,
                icp_quality,
                dropped_ratio,
            )

        reasons: List[str] = []
        if (
            diag_level is not None
            and diag_level > self.acceptance_required_diagnostic_level
        ):
            reasons.append(
                "diagnostic_level "
                f"{diag_level:.0f} exceeds "
                f"{self.acceptance_required_diagnostic_level}"
            )
        if (
            dropped_ratio is not None
            and dropped_ratio > self.acceptance_max_dropped_frames
        ):
            reasons.append(
                "dropped_frames "
                f"{dropped_ratio:.3f} exceeds "
                f"{self.acceptance_max_dropped_frames:.3f}"
            )
        if (
            icp_quality is not None
            and icp_quality < self.acceptance_min_icp_quality
        ):
            reasons.append(
                "icp_quality "
                f"{icp_quality:.3f} below "
                f"{self.acceptance_min_icp_quality:.3f}"
            )
        if (
            inlier_ratio is not None
            and inlier_ratio < self.acceptance_min_inlier_ratio
        ):
            reasons.append(
                "inlier_ratio "
                f"{inlier_ratio:.3f} below "
                f"{self.acceptance_min_inlier_ratio:.3f}"
            )
        if (
            matched_points is not None
            and self.acceptance_min_matched_points > 0
            and matched_points < self.acceptance_min_matched_points
        ):
            reasons.append(
                "matched_points "
                f"{matched_points:.0f} below "
                f"{self.acceptance_min_matched_points}"
            )
        if (
            icp_error is not None
            and icp_error > self.acceptance_max_icp_error
        ):
            reasons.append(
                "icp_error "
                f"{icp_error:.3f} exceeds "
                f"{self.acceptance_max_icp_error:.3f}"
            )

        has_direct_metric = any(
            value is not None
            for value in (
                icp_quality,
                inlier_ratio,
                icp_error,
            )
        ) or (
            matched_points is not None
            and self.acceptance_min_matched_points > 0
        )
        if reasons:
            return (
                "bad",
                "match quality below threshold: "
                + "; ".join(reasons)
                + "; "
                + metric_text,
                icp_quality,
                dropped_ratio,
            )
        if has_direct_metric:
            return (
                "ok",
                "direct match quality OK; " + metric_text,
                icp_quality,
                dropped_ratio,
            )
        if self.acceptance_allow_quality_fallback:
            return (
                "ok",
                "no direct match-quality metric; explicit fallback enabled; "
                + metric_text,
                icp_quality,
                dropped_ratio,
            )
        return (
            "pending",
            "no direct match-quality metric available and "
            "acceptance_allow_quality_fallback is false; "
            + metric_text,
            icp_quality,
            dropped_ratio,
        )

    def _pose_stability_result(
        self,
        now_sec: float,
        since_sec: float,
    ) -> Tuple[str, str]:
        if not self.acceptance_require_pose_stability:
            return "ok", "pose stability gate disabled"

        if self.last_pose_sample is None:
            return "pending", "pose stability unavailable: no pose sample"

        window_sec = self.acceptance_pose_stability_window_sec
        window_start = max(since_sec, now_sec - window_sec)
        samples = [
            sample
            for sample in self.pose_history
            if sample.time_sec >= window_start and sample.time_sec >= since_sec
        ]
        if len(samples) < 2:
            return (
                "pending",
                "pose stability pending: fewer than 2 samples in "
                f"{window_sec:.1f}s window",
            )

        span = samples[-1].time_sec - samples[0].time_sec
        if span + 1e-6 < window_sec:
            return (
                "pending",
                "pose stability pending: window span "
                f"{span:.2f}s below {window_sec:.2f}s",
            )

        anchor = samples[0]
        max_drift = 0.0
        max_yaw_drift = 0.0
        for sample in samples[1:]:
            drift = math.hypot(sample.x - anchor.x, sample.y - anchor.y)
            yaw_drift = abs(self._angle_delta(sample.yaw, anchor.yaw))
            max_drift = max(max_drift, drift)
            max_yaw_drift = max(max_yaw_drift, yaw_drift)

        if max_drift > self.acceptance_max_pose_drift_m:
            return (
                "bad",
                "pose instability: drift "
                f"{max_drift:.2f}m exceeds "
                f"{self.acceptance_max_pose_drift_m:.2f}m",
            )
        if max_yaw_drift > self.acceptance_max_yaw_drift_rad:
            return (
                "bad",
                "pose instability: yaw drift "
                f"{max_yaw_drift:.2f}rad exceeds "
                f"{self.acceptance_max_yaw_drift_rad:.2f}rad",
            )

        return (
            "ok",
            "pose stability OK: "
            f"window={span:.2f}s, drift={max_drift:.2f}m, "
            f"yaw_drift={max_yaw_drift:.2f}rad",
        )

    def _pose_covariance_result(self) -> Tuple[str, str]:
        if self.acceptance_max_pose_covariance <= 0.0:
            return "ok", "pose covariance gate disabled"
        if self.last_pose_sample is None:
            return "pending", "pose covariance unavailable: no pose sample"

        covariance = self.last_pose_sample.max_pose_covariance
        if covariance is None:
            return "ok", "pose covariance unavailable"
        if covariance > self.acceptance_max_pose_covariance:
            return (
                "bad",
                "pose covariance "
                f"{covariance:.3f} exceeds "
                f"{self.acceptance_max_pose_covariance:.3f}",
            )
        return "ok", f"pose covariance OK: {covariance:.3f}"

    def _fresh_pose_count_since(self, since_sec: Optional[float]) -> int:
        if since_sec is None:
            return 0
        return sum(
            1
            for sample in self.pose_history
            if sample.time_sec >= since_sec
        )

    def _confirmation_status(
        self,
        now_sec: float,
        promising_since: Optional[float],
    ) -> Tuple[str, str]:
        if self.acceptance_confirmation_window_sec <= 0.0:
            return "confirmed", "confirmation window disabled"

        if promising_since is None:
            return (
                "promising",
                "confirmation window started; need "
                f"{self.acceptance_confirmation_window_sec:.1f}s continuous "
                "good evidence",
            )

        age = now_sec - promising_since
        if age >= self.acceptance_confirmation_window_sec:
            return (
                "confirmed",
                "continuous good evidence for "
                f"{age:.1f}s",
            )
        return (
            "promising",
            "continuous good evidence for "
            f"{age:.1f}s/"
            f"{self.acceptance_confirmation_window_sec:.1f}s",
        )

    def _localization_healthy_for_relocalize_guard(
        self,
        now_sec: float,
    ) -> bool:
        if self.lost_active:
            return False
        return self._mola_health_ok(now_sec)

    def _mola_health_ok(self, now_sec: float) -> bool:
        if not self._pose_recent(now_sec):
            return False

        diag_status, _reason, _icp, _dropped = self._diagnostics_health_result(
            now_sec,
            None,
            require_diagnostics=False,
        )
        return diag_status == "ok"

    def _healthy_recovery_suppression_reason(self, now_sec: float) -> str:
        pose_age = (
            "none"
            if self.last_pose_time is None
            else f"{now_sec - self.last_pose_time:.2f}s"
        )
        diag_status, diag_reason, _icp, _dropped = (
            self._diagnostics_health_result(
                now_sec,
                None,
                require_diagnostics=False,
            )
        )
        return (
            f"fresh_pose_age={pose_age}; "
            f"diagnostics_status={diag_status}; {diag_reason}"
        )

    def _relocalization_blocked_by_healthy_mola(
        self,
        now_sec: float,
        context: str,
    ) -> bool:
        if not self._localization_healthy_for_relocalize_guard(now_sec):
            return False

        self._log_throttled(
            "info",
            f"{context}_mola_healthy_block",
            2.0,
            "RECOVERY_SUPPRESSED_HEALTHY reason="
            + self._healthy_recovery_suppression_reason(now_sec),
        )
        return True

    def _cancel_recovery_activity_if_healthy(self, now_sec: float) -> bool:
        if self.lost_active or not self._mola_health_ok(now_sec):
            return False

        cancelled = self._cancel_gps_yaw_sweep(
            "RECOVERY_SUPPRESSED_HEALTHY reason="
            + self._healthy_recovery_suppression_reason(now_sec)
        )
        if (
            self.pending_relocalize_future is not None
            and self.pending_relocalize_context in ("recovery", "gps_yaw_sweep")
        ):
            try:
                self.pending_relocalize_future.cancel()
            except AttributeError:
                pass
            self.pending_relocalize_future = None
            self.pending_relocalize_start = None
            self.pending_relocalize_context = ""
            cancelled = True

        if cancelled:
            self._log_throttled(
                "info",
                "recovery_cancelled_healthy",
                2.0,
                "RECOVERY_SUPPRESSED_HEALTHY reason="
                + self._healthy_recovery_suppression_reason(now_sec),
            )
        return cancelled

    def _enter_post_recovery_lockout(
        self,
        now_sec: float,
        duration_sec: float,
        reason: str,
    ) -> None:
        if duration_sec <= 0.0:
            return

        self.post_recovery_lockout_until = max(
            self.post_recovery_lockout_until,
            now_sec + duration_sec,
        )
        self.post_recovery_lockout_reason = reason
        self.get_logger().warn(
            "Post-recovery lockout entered after accepted recovery: "
            f"{reason}; lockout {duration_sec:.1f}s."
        )

    def _enter_manual_pose_lockout(self, now_sec: float, reason: str) -> None:
        if self.manual_pose_lockout_sec <= 0.0:
            return

        self.manual_pose_lockout_until = max(
            self.manual_pose_lockout_until,
            now_sec + self.manual_pose_lockout_sec,
        )
        self.get_logger().warn(
            "Auto-relocalization suppressed after manual pose acceptance: "
            f"{reason}; lockout {self.manual_pose_lockout_sec:.1f}s."
        )

    def _accept_recovery(
        self,
        now_sec: float,
        reason: str,
        active_yaw_deg: Optional[float] = None,
        manual: bool = False,
    ) -> None:
        icp_quality = self._diag_float(self.last_diag_fields, "icp_quality")
        dropped_ratio = self._diag_float(
            self.last_diag_fields,
            "dropped_frames_ratio",
        )
        yaw_text = (
            f", yaw_candidate={active_yaw_deg:.0f} deg"
            if active_yaw_deg is not None
            else ""
        )
        self.get_logger().warn(
            "Accepted recovery from MOLA/LiDAR health: "
            f"{reason}{yaw_text}; icp_quality={self._metric_text(icp_quality)}; "
            "dropped_frames_ratio="
            f"{self._metric_text(dropped_ratio)}."
        )

        self.gps_yaw_sweep_state = None
        self.lost_active = False
        self.lost_since_sec = None
        self.current_loss_reasons = []
        self.recovery_attempts_this_event = 0
        self.gps_yaw_sweep_failed_event_id = 0
        self.localization_good_since = now_sec
        self.logged_healthy = True
        self.manual_pose_pending = False
        self.manual_pose_promising_since = None
        self.recovery_promising_since = None

        if self.last_pose_sample is not None:
            self.last_good_pose = self._pose_msg_from_sample(
                self.last_pose_sample,
                self.recovery_xy_std,
                self.recovery_yaw_std,
            )
            self.last_good_pose_time = now_sec

        if manual:
            self._enter_manual_pose_lockout(now_sec, reason)
        else:
            self._enter_post_recovery_lockout(
                now_sec,
                self.post_recovery_lockout_sec,
                reason,
            )

    def _update_healthy_state(self, now_sec: float) -> None:
        if not self._pose_recent(now_sec):
            self.localization_good_since = None
            self.recovery_promising_since = None
            self.logged_healthy = False
            return

        if self.bad_diag_since is not None:
            self.localization_good_since = None
            self.recovery_promising_since = None
            self.logged_healthy = False
            return

        if (
            self.manual_pose_pending
            and self.manual_pose_time is not None
            and self.last_pose_time is not None
            and self.last_pose_time >= self.manual_pose_time
        ):
            fresh_pose_count = self._fresh_pose_count_since(
                self.manual_pose_time
            )
            evidence_status, evidence_reason, _icp, _dropped = (
                self._acceptance_evidence_result(
                    now_sec,
                    self.manual_pose_time,
                    fresh_pose_count,
                    seed_xy=None,
                )
            )
            if evidence_status != "ok":
                cooldown_active = self._manual_pose_cooldown_active(now_sec)
                self.manual_pose_promising_since = None
                self._log_throttled(
                    "info",
                    "manual_pose_not_accepted_yet",
                    2.0,
                    "Manual pose has some MOLA evidence but is not confirmed "
                    f"healthy yet ({evidence_reason}); not accepted yet.",
                )
                if cooldown_active:
                    return
                self.manual_pose_pending = False
                self.get_logger().warn(
                    "Manual pose cooldown expired without healthy MOLA "
                    "evidence; no manual lockout entered."
                )
            else:
                confirm_status, confirm_reason = self._confirmation_status(
                    now_sec,
                    self.manual_pose_promising_since,
                )
                if self.manual_pose_promising_since is None:
                    self.manual_pose_promising_since = now_sec
                if confirm_status != "confirmed":
                    self._log_throttled(
                        "info",
                        "manual_pose_promising",
                        1.0,
                        "Manual pose is promising but not confirmed yet: "
                        f"{confirm_reason}; {evidence_reason}.",
                    )
                    return
                self._accept_recovery(
                    now_sec,
                    "manual pose confirmed by MOLA/LiDAR evidence: "
                    + evidence_reason,
                    manual=True,
                )
                return

        if self._manual_pose_cooldown_active(now_sec):
            cooldown_left = self.manual_pose_cooldown_until - now_sec
            self.localization_good_since = None
            self.logged_healthy = False
            self._log_throttled(
                "info",
                "manual_pose_cooldown_wait",
                2.0,
                "Manual pose cooldown active; auto-recovery suppressed "
                f"({cooldown_left:.1f}s left).",
            )
            return

        if (
            self.manual_pose_pending
            and self.manual_pose_time is not None
            and now_sec > self.manual_pose_cooldown_until
        ):
            self.manual_pose_pending = False
            self.manual_pose_promising_since = None
            self.get_logger().warn(
                "Manual pose cooldown expired without healthy MOLA evidence; "
                "no manual lockout entered."
            )

        if self.localization_good_since is None:
            self.localization_good_since = now_sec
            self._log_throttled(
                "info",
                "good_pose_candidate",
                5.0,
                "Good MOLA pose candidate started; waiting "
                f"{self.min_good_pose_age_sec:.1f}s before saving "
                "last_good_pose.",
            )
            return

        good_age = now_sec - self.localization_good_since
        if good_age < self.min_good_pose_age_sec:
            return

        if self.lost_active:
            fresh_pose_count = self._fresh_pose_count_since(
                self.localization_good_since
            )
            evidence_status, evidence_reason, _icp, _dropped = (
                self._acceptance_evidence_result(
                    now_sec,
                    self.localization_good_since,
                    fresh_pose_count,
                    seed_xy=None,
                )
            )
            if evidence_status != "ok":
                self.recovery_promising_since = None
                self._log_throttled(
                    "info",
                    "recovery_not_confirmed_yet",
                    1.0,
                    "Recovery pose is not confirmed yet: "
                    f"{evidence_reason}.",
                )
                return

            confirm_status, confirm_reason = self._confirmation_status(
                now_sec,
                self.recovery_promising_since,
            )
            if self.recovery_promising_since is None:
                self.recovery_promising_since = now_sec
            if confirm_status != "confirmed":
                self._log_throttled(
                    "info",
                    "recovery_promising",
                    1.0,
                    "Recovery pose is promising but not confirmed yet: "
                    f"{confirm_reason}; {evidence_reason}.",
                )
                return

            self._accept_recovery(
                now_sec,
                "recovery confirmed by MOLA/LiDAR evidence after "
                f"{good_age:.1f}s: {evidence_reason}",
            )
            return

        if not self.logged_healthy:
            self.get_logger().info("Localization healthy.")
            self.logged_healthy = True

        if self.last_pose_sample is not None:
            had_good_pose = self.last_good_pose is not None
            self.last_good_pose = self._pose_msg_from_sample(
                self.last_pose_sample,
                self.recovery_xy_std,
                self.recovery_yaw_std,
            )
            self.last_good_pose_time = now_sec
            if not had_good_pose:
                self.get_logger().info(
                    "Promoted settled MOLA pose to last_good_pose."
                )

    def _maybe_startup_relocalize(self, now_sec: float) -> None:
        if not self.enable_startup_relocalize:
            return
        if self._relocalization_blocked_by_healthy_mola(now_sec, "startup"):
            return
        if self._auto_relocalize_lockout_active(now_sec):
            self._log_auto_relocalize_suppressed(now_sec, "startup")
            return
        if self._manual_pose_cooldown_active(now_sec):
            cooldown_left = self.manual_pose_cooldown_until - now_sec
            self._log_throttled(
                "info",
                "startup_manual_pose_cooldown",
                2.0,
                "Startup relocalization skipped during manual pose cooldown "
                f"({cooldown_left:.1f}s left).",
            )
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
        if not self.enable_recovery_relocalize:
            return
        if self._relocalization_blocked_by_healthy_mola(now_sec, "recovery"):
            return
        if not self.lost_active:
            if self.use_gps_relocalize:
                self._log_throttled(
                    "info",
                    "gps_recovery_suppressed_healthy",
                    15.0,
                    "GPS auto-recovery suppressed because MOLA localization "
                    "is healthy/monitoring only.",
                )
            return
        if not self._recovery_sustained_loss_ready(now_sec):
            return
        if self._auto_relocalize_lockout_active(now_sec):
            self._cancel_gps_yaw_sweep(
                "Auto-recovery lockout active; cancelling GPS yaw sweep."
            )
            self._log_auto_relocalize_suppressed(now_sec, "recovery")
            return
        if self._manual_pose_cooldown_active(now_sec):
            cooldown_left = self.manual_pose_cooldown_until - now_sec
            self._cancel_gps_yaw_sweep(
                "Manual pose cooldown active; cancelling GPS yaw sweep / "
                "suppressing recovery during cooldown."
            )
            self._log_throttled(
                "info",
                "recovery_manual_pose_cooldown",
                2.0,
                "Recovery relocalization skipped during manual pose cooldown "
                f"({cooldown_left:.1f}s left).",
            )
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

        if (
            self.gps_yaw_sweep_enabled
            and self.use_gps_relocalize
            and self.gps_yaw_sweep_failed_event_id != self.loss_event_id
        ):
            if self._start_gps_yaw_sweep(now_sec):
                self.recovery_attempts_this_event += 1
                return

        requested = self._request_relocalize(
            "recovery",
            "localization suspected lost",
            use_startup_std=False,
        )
        if requested:
            self.recovery_attempts_this_event += 1

    def _recovery_sustained_loss_ready(self, now_sec: float) -> bool:
        if not self.recovery_require_sustained_loss:
            return True

        if self.lost_since_sec is None:
            self.lost_since_sec = now_sec

        lost_age = now_sec - self.lost_since_sec
        if lost_age >= self.recovery_min_lost_duration_sec:
            return True

        self._log_throttled(
            "info",
            "recovery_wait_sustained_loss",
            1.0,
            "Recovery waiting for sustained localization loss "
            f"({lost_age:.1f}s/"
            f"{self.recovery_min_lost_duration_sec:.1f}s).",
        )
        return False

    def _log_auto_relocalize_suppressed(
        self,
        now_sec: float,
        context: str,
    ) -> None:
        reasons: List[str] = []
        if self._manual_pose_cooldown_active(now_sec):
            reasons.append(
                "manual pose cooldown "
                f"({self.manual_pose_cooldown_until - now_sec:.1f}s left)"
            )
        if self._manual_pose_lockout_active(now_sec):
            reasons.append(
                "manual pose lockout "
                f"({self.manual_pose_lockout_until - now_sec:.1f}s left)"
            )
        if self._post_recovery_lockout_active(now_sec):
            detail = self.post_recovery_lockout_reason or "post recovery"
            reasons.append(
                f"{detail} lockout "
                f"({self.post_recovery_lockout_until - now_sec:.1f}s left)"
            )

        reason_text = "; ".join(reasons) if reasons else "lockout active"
        self._log_throttled(
            "info",
            f"{context}_auto_recovery_lockout",
            2.0,
            "Auto-relocalization suppressed: " + reason_text + ".",
        )

    def _request_relocalize(
        self,
        kind: str,
        reason: str,
        use_startup_std: bool,
    ) -> bool:
        now_sec = self._now_sec()
        selected_pose = self._select_relocalize_pose(
            kind,
            use_startup_std,
            now_sec,
        )
        if selected_pose is None:
            return False
        pose_msg, pose_source, xy_std, z_std, yaw_std = selected_pose

        return self._request_relocalize_with_pose(
            kind,
            reason,
            pose_msg,
            pose_source,
            xy_std,
            z_std,
            yaw_std,
        )

    def _request_relocalize_with_pose(
        self,
        kind: str,
        reason: str,
        pose_msg: PoseWithCovarianceStamped,
        pose_source: str,
        xy_std: float,
        z_std: float,
        yaw_std: float,
        ignore_cooldown: bool = False,
        update_cooldown: bool = True,
    ) -> bool:
        now_sec = self._now_sec()
        if self._relocalization_blocked_by_healthy_mola(now_sec, kind):
            return False
        if self._auto_relocalize_lockout_active(now_sec):
            self._log_auto_relocalize_suppressed(now_sec, kind)
            return False

        if self._manual_pose_cooldown_active(now_sec):
            cooldown_left = self.manual_pose_cooldown_until - now_sec
            self._log_throttled(
                "info",
                "request_manual_pose_cooldown",
                2.0,
                "Relocalization request skipped during manual pose cooldown "
                f"({cooldown_left:.1f}s left).",
            )
            return False

        if self.pending_relocalize_future is not None:
            self._log_throttled(
                "info",
                "relocalize_pending",
                5.0,
                "Relocalization request already pending.",
            )
            return False

        cooldown_sec = self._relocalize_cooldown_sec_for(kind, pose_source)
        if not ignore_cooldown and self.last_relocalize_time is not None:
            cooldown_left = (
                cooldown_sec
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

        request = self.relocalize_srv_type.Request()
        assigned = self._populate_relocalize_request(
            request,
            pose_msg,
            xy_std,
            z_std,
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
        if update_cooldown:
            self.last_relocalize_time = now_sec

        pose = pose_msg.pose.pose
        yaw = self._yaw_from_pose(pose)
        if self._pose_source_is_gps(pose_source):
            self.get_logger().warn(
                "GPS seed relocalization attempt near map "
                f"x={pose.position.x:.2f}, y={pose.position.y:.2f}, "
                f"yaw={math.degrees(yaw):.1f} deg. "
                "GPS is only a seed; MOLA/LiDAR health will validate "
                "the final localization."
            )

        self.get_logger().warn(
            "Relocalization requested: "
            f"{reason}; using {pose_source}; "
            f"service_type={self.relocalize_srv_type_name}; "
            f"xy_std={xy_std:.2f}, z_std={z_std:.2f}, "
            f"yaw_std={yaw_std:.2f}."
        )
        return True

    def _relocalize_cooldown_sec_for(
        self,
        kind: str,
        pose_source: str,
    ) -> float:
        if kind in ("recovery", "gps_yaw_sweep") and self._pose_source_is_gps(
            pose_source
        ):
            return self.gps_relocalize_cooldown_sec
        return self.relocalize_cooldown_sec

    def _active_relocalize_cooldown_sec(self) -> float:
        if self.use_gps_relocalize:
            return self.gps_relocalize_cooldown_sec
        return self.relocalize_cooldown_sec

    def _pose_source_is_gps(self, pose_source: str) -> bool:
        return pose_source.strip().lower().startswith("gps")

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
        z_std: float,
        yaw_std: float,
    ) -> List[str]:
        assigned: List[str] = []
        field_types = self._field_types(request)

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
            z_std,
            yaw_std,
        )
        for field_name, value in numeric_values.items():
            if hasattr(request, field_name):
                if self._try_setattr(request, field_name, value):
                    assigned.append(field_name)

        if hasattr(request, "covariance"):
            covariance = self._covariance_from_std(xy_std, yaw_std, z_std)
            if self._try_setattr(request, "covariance", covariance):
                assigned.append("covariance")

        return assigned

    def _numeric_request_values(
        self,
        pose_msg: PoseWithCovarianceStamped,
        xy_std: float,
        z_std: float,
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
            "z_std": z_std,
            "std_z": z_std,
            "sigma_z": z_std,
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
        if kind == "gps_yaw_sweep" and self.gps_yaw_sweep_state is not None:
            self.gps_yaw_sweep_state.candidate_service_success = success
            self.gps_yaw_sweep_state.candidate_service_message = message
        if success:
            if kind in ("recovery", "gps_yaw_sweep"):
                message_suffix = f": {message}." if message else "."
                self.get_logger().info(
                    "Relocalization request accepted"
                    f"{message_suffix} "
                    "Waiting for fresh MOLA pose/diagnostics to validate "
                    "recovery."
                )
            else:
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

    def _start_gps_yaw_sweep(self, now_sec: float) -> bool:
        if self._relocalization_blocked_by_healthy_mola(
            now_sec,
            "gps_yaw_sweep",
        ):
            return False

        seed, reason = self._gps_recovery_seed(now_sec)
        if seed is None:
            if self.last_good_pose is not None:
                self._log_throttled(
                    "info",
                    "gps_yaw_sweep_unavailable",
                    5.0,
                    "GPS yaw sweep unavailable "
                    f"({reason}); using last-known-pose seed.",
                )
            return False

        map_x, map_y, z, _fix_age = seed
        map_seed = self.latest_gps_map_seed
        assert map_seed is not None
        angles = self._gps_yaw_sweep_candidate_angles()
        self.recovery_episode_id += 1
        self.gps_yaw_sweep_state = GpsYawSweepState(
            map_x=map_x,
            map_y=map_y,
            z=z,
            seed_time_sec=map_seed.time_sec,
            seed_fix_time_sec=map_seed.fix_time_sec,
            seed_source="GPS map seed",
            yaw_mode=self.gps_yaw_sweep_mode,
            base_yaw_rad=map_seed.yaw,
            angles_deg=angles,
        )
        self.get_logger().warn(
            "Starting GPS recovery episode "
            f"{self.recovery_episode_id}; frozen seed "
            f"x={map_x:.2f}, y={map_y:.2f}; trying "
            f"{len(angles)} candidate(s), yaw_mode="
            f"{self.gps_yaw_sweep_mode}. GPS x/y will not move during "
            "this yaw sweep."
        )
        self._log_gps_seed_conversion(map_seed, "frozen_yaw_sweep_seed")
        self._process_gps_yaw_sweep(now_sec)
        return True

    def _process_gps_yaw_sweep(self, now_sec: float) -> None:
        state = self.gps_yaw_sweep_state
        if state is None:
            return

        if self._manual_pose_cooldown_active(now_sec):
            self._cancel_gps_yaw_sweep(
                "Manual pose cooldown active; cancelling GPS yaw sweep / "
                "suppressing recovery during cooldown."
            )
            return

        if self.pending_relocalize_future is not None:
            return

        if not self.lost_active:
            self._cancel_gps_yaw_sweep(
                "MOLA is no longer marked lost; cancelling GPS yaw sweep "
                "without post-recovery lockout."
            )
            return

        if state.candidate_sent_time is not None:
            age = now_sec - state.candidate_sent_time
            if age < self.yaw_candidate_settle_sec:
                if not state.candidate_wait_logged:
                    state.candidate_wait_logged = True
                    self.get_logger().info(
                        "Waiting for GPS yaw candidate "
                        f"{state.active_yaw_deg:.0f} deg to settle "
                        f"({self.yaw_candidate_settle_sec:.1f}s window)."
                    )
                return

            result, reason, icp_quality, dropped_ratio = (
                self._gps_yaw_sweep_candidate_result(state, now_sec)
            )
            if result == "accepted":
                self._accept_recovery(
                    now_sec,
                    reason,
                    active_yaw_deg=state.active_yaw_deg,
                )
                return

            if result == "pending":
                self._log_throttled(
                    "info",
                    "gps_yaw_candidate_pending",
                    1.0,
                    "Waiting for GPS yaw candidate "
                    f"{state.active_yaw_deg:.0f} deg evaluation: "
                    f"{reason}.",
                )
                return

            self.get_logger().warn(
                "Rejected GPS yaw candidate "
                f"{state.active_yaw_deg:.0f} deg; reason={reason}; "
                f"icp_quality={self._metric_text(icp_quality)}; "
                "dropped_frames_ratio="
                f"{self._metric_text(dropped_ratio)}. "
                "No lockout entered because candidate was not accepted."
            )
            state.active_yaw_deg = None
            state.active_yaw_rad = None
            state.candidate_sent_time = None
            state.candidate_pose_sequence = 0
            state.candidate_wait_logged = False
            state.candidate_service_success = None
            state.candidate_service_message = ""
            state.candidate_promising_since = None
            state.candidate_promising_reason = ""

        self._try_next_gps_yaw_candidate(now_sec)

    def _try_next_gps_yaw_candidate(self, now_sec: float) -> None:
        state = self.gps_yaw_sweep_state
        if state is None:
            return

        if state.next_index >= len(state.angles_deg):
            self._gps_yaw_sweep_failed()
            return

        if state.attempts >= self.gps_yaw_sweep_max_attempts:
            self._gps_yaw_sweep_failed()
            return

        if not self._cloud_ready(now_sec):
            self._log_throttled(
                "info",
                "gps_yaw_sweep_wait_cloud",
                5.0,
                "GPS yaw sweep waiting for a recent, dense LiDAR cloud.",
            )
            return

        yaw_deg = state.angles_deg[state.next_index]
        yaw_rad = self._gps_yaw_candidate_rad(state, yaw_deg)
        pose_msg = self._gps_recovery_pose_from_map(
            state.map_x,
            state.map_y,
            state.z,
            yaw_rad,
            self.gps_yaw_sweep_candidate_yaw_std,
        )

        self.get_logger().warn(
            "Trying GPS yaw candidate "
            f"{yaw_deg:.0f} deg ({state.yaw_mode}; actual "
            f"{math.degrees(yaw_rad):.1f} deg) at frozen map "
            f"x={state.map_x:.2f}, y={state.map_y:.2f}."
        )
        requested = self._request_relocalize_with_pose(
            "gps_yaw_sweep",
            f"GPS yaw sweep candidate {yaw_deg:.0f} deg",
            pose_msg,
            f"GPS yaw candidate {yaw_deg:.0f} deg",
            self.gps_recovery_xy_std,
            self.gps_recovery_z_std,
            self.gps_yaw_sweep_candidate_yaw_std,
            ignore_cooldown=True,
            update_cooldown=False,
        )
        if not requested:
            return

        state.active_yaw_deg = yaw_deg
        state.active_yaw_rad = yaw_rad
        state.candidate_sent_time = self._now_sec()
        state.candidate_pose_sequence = self.pose_sequence
        state.candidate_wait_logged = False
        state.candidate_service_success = None
        state.candidate_service_message = ""
        state.candidate_promising_since = None
        state.candidate_promising_reason = ""
        state.next_index += 1
        state.attempts += 1

    def _gps_yaw_candidate_rad(
        self,
        state: GpsYawSweepState,
        yaw_deg: float,
    ) -> float:
        if state.yaw_mode == "relative":
            return self._normalize_angle(
                state.base_yaw_rad + math.radians(yaw_deg)
            )
        return self._normalize_angle(math.radians(yaw_deg))

    def _gps_yaw_sweep_candidate_result(
        self,
        state: GpsYawSweepState,
        now_sec: float,
    ) -> Tuple[str, str, Optional[float], Optional[float]]:
        reasons: List[str] = []
        timed_out = False
        if state.candidate_sent_time is not None:
            timed_out = (
                now_sec - state.candidate_sent_time
                >= self.yaw_candidate_timeout_sec
            )

        if state.candidate_service_success is False:
            message = state.candidate_service_message
            reasons.append(
                "relocalize service returned failure"
                f"{': ' + message if message else ''}"
            )

        if state.candidate_sent_time is None:
            reasons.append("candidate request time unavailable")

        if state.candidate_sent_time is None:
            icp_quality = self._diag_first_float(
                self.last_diag_fields,
                ["icp_quality", "match_quality"],
            )
            dropped_ratio = self._diag_first_float(
                self.last_diag_fields,
                ["dropped_frames_ratio", "dropped_frames"],
            )
        else:
            fresh_pose_count = max(
                0,
                self.pose_sequence - state.candidate_pose_sequence,
            )
            evidence_status, evidence_reason, icp_quality, dropped_ratio = (
                self._acceptance_evidence_result(
                    now_sec,
                    state.candidate_sent_time,
                    fresh_pose_count,
                    seed_xy=(state.map_x, state.map_y),
                )
            )
            if evidence_status == "bad":
                reasons.append(evidence_reason)
            elif evidence_status == "pending":
                state.candidate_promising_since = None
                state.candidate_promising_reason = ""
                if timed_out:
                    reasons.append(
                        "candidate did not become confirmed before timeout: "
                        + evidence_reason
                    )
                else:
                    return (
                        "pending",
                        evidence_reason,
                        icp_quality,
                        dropped_ratio,
                    )
            else:
                if state.candidate_promising_since is None:
                    state.candidate_promising_since = now_sec
                    state.candidate_promising_reason = evidence_reason
                confirm_status, confirm_reason = self._confirmation_status(
                    now_sec,
                    state.candidate_promising_since,
                )
                if confirm_status == "confirmed":
                    return (
                        "accepted",
                        "CANDIDATE_CONFIRMED: "
                        f"{confirm_reason}; {evidence_reason}",
                        icp_quality,
                        dropped_ratio,
                    )
                if timed_out:
                    state.candidate_promising_since = None
                    state.candidate_promising_reason = ""
                    reasons.append(
                        "candidate did not become confirmed before timeout: "
                        f"{confirm_reason}; {evidence_reason}"
                    )
                else:
                    return (
                        "pending",
                        "CANDIDATE_PROMISING: "
                        f"{confirm_reason}; {evidence_reason}",
                        icp_quality,
                        dropped_ratio,
                    )

        if reasons:
            return "rejected", "; ".join(reasons), icp_quality, dropped_ratio
        return "pending", "candidate evidence unavailable", icp_quality, dropped_ratio

    def _gps_yaw_sweep_failed(self) -> None:
        self.gps_yaw_sweep_state = None
        self.gps_yaw_sweep_failed_event_id = self.loss_event_id
        self.get_logger().warn(
            "GPS yaw sweep exhausted without accepted MOLA/LiDAR recovery; "
            "falling back to last-known-pose seed if available."
        )

        if self.last_good_pose is None:
            self.get_logger().warn(
                "GPS yaw sweep failed; no last-known-pose seed is available."
            )
            return

        requested = self._request_relocalize_with_pose(
            "recovery",
            "GPS yaw sweep failed",
            self.last_good_pose,
            "last-known-pose seed",
            self.recovery_xy_std,
            self.recovery_xy_std,
            self.recovery_yaw_std,
            ignore_cooldown=True,
        )
        if requested:
            self.recovery_attempts_this_event += 1

    def _cancel_gps_yaw_sweep(self, reason: str) -> bool:
        had_sweep = self.gps_yaw_sweep_state is not None or (
            self.pending_relocalize_future is not None
            and self.pending_relocalize_context == "gps_yaw_sweep"
        )

        if had_sweep:
            self.get_logger().warn(reason)

        self.gps_yaw_sweep_state = None

        if (
            self.pending_relocalize_future is not None
            and self.pending_relocalize_context == "gps_yaw_sweep"
        ):
            try:
                self.pending_relocalize_future.cancel()
            except AttributeError:
                pass
            self.pending_relocalize_future = None
            self.pending_relocalize_start = None
            self.pending_relocalize_context = ""

        return had_sweep

    def _gps_yaw_sweep_candidate_angles(self) -> List[float]:
        return self.gps_yaw_sweep_angles_deg[
            : self.gps_yaw_sweep_max_attempts
        ]

    def _metric_text(self, value: Optional[float]) -> str:
        if value is None:
            return "unknown"
        return f"{value:.3f}"

    def _select_relocalize_pose(
        self,
        kind: str,
        use_startup_std: bool,
        now_sec: float,
    ) -> Optional[Tuple[PoseWithCovarianceStamped, str, float, float, float]]:
        gps_reason = ""
        gps_sweep_failed = (
            self.gps_yaw_sweep_enabled
            and self.gps_yaw_sweep_failed_event_id == self.loss_event_id
        )
        if kind == "recovery" and self.use_gps_relocalize and not gps_sweep_failed:
            gps_pose, gps_reason = self._gps_recovery_pose_msg(now_sec)
            if gps_pose is not None:
                return (
                    gps_pose,
                    "GPS seed",
                    self.gps_recovery_xy_std,
                    self.gps_recovery_z_std,
                    self.gps_recovery_yaw_std,
                )

            if self.last_good_pose is not None:
                self._log_throttled(
                    "info",
                    "gps_seed_fallback_last_good",
                    5.0,
                    "GPS recovery seed unavailable "
                    f"({gps_reason}); using last-known-pose seed.",
                )

        if self.last_good_pose is not None:
            xy_std = (
                self.startup_xy_std
                if use_startup_std
                else self.recovery_xy_std
            )
            yaw_std = (
                self.startup_yaw_std
                if use_startup_std
                else self.recovery_yaw_std
            )
            return (
                self.last_good_pose,
                "last-known-pose seed",
                xy_std,
                xy_std,
                yaw_std,
            )

        if kind == "recovery":
            if not gps_reason:
                gps_reason = (
                    "GPS relocalization disabled"
                    if not self.use_gps_relocalize
                    else (
                        "GPS yaw sweep already failed for this loss event"
                        if gps_sweep_failed
                        else self._gps_unavailable_reason(now_sec)
                    )
                )
            self._log_throttled(
                "warn",
                "recovery_no_seed",
                5.0,
                "Recovery relocalization skipped: no GPS seed "
                f"({gps_reason}) and no settled last_good_pose exists yet.",
            )
            return None

        return (
            self._startup_pose_msg(),
            "configured startup seed",
            self.startup_xy_std,
            self.startup_xy_std,
            self.startup_yaw_std,
        )

    def _gps_recovery_pose_msg(
        self,
        now_sec: float,
    ) -> Tuple[Optional[PoseWithCovarianceStamped], str]:
        seed, reason = self._gps_recovery_seed(now_sec)
        if seed is None:
            return None, reason

        map_x, map_y, z, seed_age = seed
        map_seed = self.latest_gps_map_seed
        assert map_seed is not None
        msg = self._gps_recovery_pose_from_map(
            map_x,
            map_y,
            z,
            map_seed.yaw,
            self.gps_recovery_yaw_std,
        )

        self._log_gps_seed_conversion(map_seed, "selected_recovery_seed")
        self._log_throttled(
            "info",
            "gps_seed_selected",
            2.0,
            "Recovery relocalization selected GPS seed "
            f"(lat={map_seed.latitude:.8f}, "
            f"lon={map_seed.longitude:.8f}, "
            f"map_x={map_x:.2f}, map_y={map_y:.2f}, "
            f"age={seed_age:.1f}s). GPS narrows the search; "
            "MOLA/LiDAR validates recovery.",
        )
        return msg, ""

    def _gps_recovery_seed(
        self,
        now_sec: float,
    ) -> Tuple[Optional[Tuple[float, float, float, float]], str]:
        unavailable_reason = self._gps_unavailable_reason(now_sec)
        if unavailable_reason:
            return None, unavailable_reason

        assert self.latest_gps_map_seed is not None
        seed = self.latest_gps_map_seed
        seed_age = now_sec - seed.time_sec
        self._log_throttled(
            "info",
            "gps_recovery_seed_ready",
            5.0,
            "GPS recovery seed ready near map "
            f"x={seed.map_x:.2f}, y={seed.map_y:.2f}, "
            f"seed_age={seed_age:.1f}s."
        )
        return (seed.map_x, seed.map_y, seed.z, seed_age), ""

    def _gps_recovery_pose_from_map(
        self,
        map_x: float,
        map_y: float,
        z: float,
        yaw: float,
        yaw_std: float,
    ) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = map_x
        msg.pose.pose.position.y = map_y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation = self._quaternion_from_yaw(yaw)
        msg.pose.covariance = self._covariance_from_std(
            self.gps_recovery_xy_std,
            yaw_std,
            self.gps_recovery_z_std,
        )
        return msg

    def _gps_unavailable_reason(self, now_sec: float) -> str:
        if not self.use_gps_relocalize:
            return "GPS relocalization disabled"
        if self.last_gps_fix is None:
            return "no valid GPS fix received"
        if self.latest_gps_map_seed is None:
            return "no GPS fix has converted to a map seed yet"

        age = now_sec - self.last_gps_fix.time_sec
        if age > self.gps_stale_timeout_sec:
            return (
                f"GPS fix stale for {age:.1f}s "
                f"(timeout {self.gps_stale_timeout_sec:.1f}s)"
            )
        seed_age = now_sec - self.latest_gps_map_seed.time_sec
        if seed_age > self.gps_seed_max_age_sec:
            return (
                f"GPS map seed stale for {seed_age:.1f}s "
                f"(timeout {self.gps_seed_max_age_sec:.1f}s)"
            )
        return ""

    def _gps_to_map_xy(self, fix: GpsFixSample) -> Tuple[float, float]:
        source = self.gps_to_map_source
        if source in ("", "landmark", "landmarks"):
            source = "landmark_calibration"

        if source not in ("landmark_calibration", "calibration_file"):
            raise ValueError(
                "unsupported gps_to_map_source "
                f"'{self.gps_to_map_source}'"
            )

        calibration = self._ensure_gps_to_map_calibration()
        if calibration is None:
            raise ValueError("GPS-to-map calibration unavailable")

        map_x, map_y = self._gps_to_local(
            fix.latitude,
            fix.longitude,
            *calibration,
        )
        if not math.isfinite(map_x) or not math.isfinite(map_y):
            raise ValueError("GPS-to-map conversion produced non-finite XY")
        return float(map_x), float(map_y)

    def _ensure_gps_to_map_calibration(self) -> Optional[Tuple[float, ...]]:
        if self.gps_to_map_config_loaded:
            return self.gps_to_map_calibration

        self.gps_to_map_config_loaded = True
        config_path = self._gps_calibration_path()
        if not config_path:
            self.gps_to_map_method = "unavailable_no_calibration_config"
            self._log_throttled(
                "warn",
                "gps_calibration_missing",
                10.0,
                "GPS relocalization enabled but no calibration config path "
                "was provided.",
            )
            return None

        helper_error = ""
        try:
            from navigation import simple_gps_util
        except ImportError as exc:
            helper_error = str(exc)
            simple_gps_util = None

        try:
            if simple_gps_util is not None:
                local_points, gps_points = (
                    simple_gps_util.load_landmark_calibration(config_path)
                )
                self.gps_to_map_calibration = (
                    simple_gps_util.calibrate_with_landmarks(
                        local_points,
                        gps_points,
                    )
                )
                self.gps_to_map_method = (
                    "navigation.simple_gps_util.gps_to_local"
                )
            else:
                self._log_throttled(
                    "warn",
                    "gps_calibration_helper_unavailable",
                    30.0,
                    "navigation.simple_gps_util unavailable "
                    f"({helper_error}); using built-in GPS calibration.",
                )
                local_points, gps_points = self._load_landmark_calibration(
                    config_path
                )
                self.gps_to_map_calibration = (
                    self._calibrate_with_landmarks(
                        local_points,
                        gps_points,
                    )
                )
                self.gps_to_map_method = (
                    "built_in_landmark_calibration_gps_to_local"
                )
            self.gps_to_map_config_path = config_path
        except (AttributeError, OSError, ValueError) as exc:
            self.get_logger().warn(
                "Failed to load GPS-to-map calibration from "
                f"{config_path}: {exc}"
            )
            self.gps_to_map_calibration = None
            self.gps_to_map_method = "calibration_load_failed"
            return None

        self.get_logger().info(
            "Loaded GPS-to-map landmark calibration from "
            f"{self.gps_to_map_config_path}."
        )
        return self.gps_to_map_calibration

    def _gps_calibration_path(self) -> str:
        if self.gps_calibration_config_path:
            return os.path.abspath(
                os.path.expanduser(self.gps_calibration_config_path)
            )

        if self.calibration_config_dir and self.calibration_config_file:
            return os.path.abspath(
                os.path.join(
                    os.path.expanduser(self.calibration_config_dir),
                    self.calibration_config_file,
                )
            )

        return ""

    def _load_landmark_calibration(
        self,
        config_path: str,
    ) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
        if yaml is None:
            raise ValueError("PyYAML is required to load GPS calibration")

        with open(config_path, "r", encoding="utf-8") as config_stream:
            config = yaml.safe_load(config_stream) or {}

        landmarks = config.get("landmarks")
        if not isinstance(landmarks, list) or len(landmarks) < 2:
            raise ValueError(
                "calibration config must define at least 2 landmarks"
            )

        local_points: List[Tuple[float, float]] = []
        gps_points: List[Tuple[float, float]] = []
        for index, landmark in enumerate(landmarks):
            try:
                local = landmark["local"]
                gps = landmark["gps"]
                local_points.append((float(local["x"]), float(local["y"])))
                gps_points.append(
                    (
                        float(gps["latitude"]),
                        float(gps["longitude"]),
                    )
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"invalid landmark entry at index {index}"
                ) from exc

        return local_points, gps_points

    def _calibrate_with_landmarks(
        self,
        local_points: List[Tuple[float, float]],
        gps_points: List[Tuple[float, float]],
    ) -> Tuple[float, ...]:
        if len(local_points) != len(gps_points):
            raise ValueError("local and GPS landmark counts differ")
        if len(local_points) < 2:
            raise ValueError("at least 2 landmarks are required")

        ref_lat = sum(lat for lat, _ in gps_points) / len(gps_points)
        ref_lon = sum(lon for _, lon in gps_points) / len(gps_points)
        gps_meters = [
            self._latlon_to_xy(lat, lon, ref_lat, ref_lon)
            for lat, lon in gps_points
        ]

        cx_local = sum(x for x, _ in local_points) / len(local_points)
        cy_local = sum(y for _, y in local_points) / len(local_points)
        cx_gps = sum(x for x, _ in gps_meters) / len(gps_meters)
        cy_gps = sum(y for _, y in gps_meters) / len(gps_meters)

        dot_sum = 0.0
        cross_sum = 0.0
        for gps_point, local_point in zip(gps_meters, local_points):
            gps_dx = gps_point[0] - cx_gps
            gps_dy = gps_point[1] - cy_gps
            local_dx = local_point[0] - cx_local
            local_dy = local_point[1] - cy_local
            dot_sum += gps_dx * local_dx + gps_dy * local_dy
            cross_sum += gps_dx * local_dy - gps_dy * local_dx

        if abs(dot_sum) < 1e-12 and abs(cross_sum) < 1e-12:
            raise ValueError("landmark calibration is degenerate")

        theta_degrees = math.degrees(math.atan2(cross_sum, dot_sum))
        return (
            ref_lat,
            ref_lon,
            cx_local,
            cy_local,
            cx_gps,
            cy_gps,
            theta_degrees,
        )

    def _gps_to_local(
        self,
        lat: float,
        lon: float,
        ref_lat: float,
        ref_lon: float,
        cx_local: float,
        cy_local: float,
        cx_gps: float,
        cy_gps: float,
        theta_degrees: float,
    ) -> Tuple[float, float]:
        x_m, y_m = self._latlon_to_xy(lat, lon, ref_lat, ref_lon)
        dx = x_m - cx_gps
        dy = y_m - cy_gps

        theta_rad = math.radians(theta_degrees)
        cos_t = math.cos(theta_rad)
        sin_t = math.sin(theta_rad)
        rot_dx = dx * cos_t - dy * sin_t
        rot_dy = dx * sin_t + dy * cos_t

        return rot_dx + cx_local, rot_dy + cy_local

    def _latlon_to_xy(
        self,
        lat: float,
        lon: float,
        lat0: float,
        lon0: float,
    ) -> Tuple[float, float]:
        x = (lon - lon0) * self._meters_per_degree_lon(lat0)
        y = (lat - lat0) * self._meters_per_degree_lat(lat0)
        return x, y

    def _meters_per_degree_lon(self, lat0: float) -> float:
        lat0rad = math.radians(lat0)
        return (
            111415.13 * math.cos(lat0rad)
            - 94.55 * math.cos(3.0 * lat0rad)
            - 0.12 * math.cos(5.0 * lat0rad)
        )

    def _meters_per_degree_lat(self, lat0: float) -> float:
        lat0rad = math.radians(lat0)
        return (
            111132.09
            - 566.05 * math.cos(2.0 * lat0rad)
            + 1.20 * math.cos(4.0 * lat0rad)
            - 0.002 * math.cos(6.0 * lat0rad)
        )

    def _gps_seed_z(self) -> float:
        if self.last_good_pose is not None:
            return float(self.last_good_pose.pose.pose.position.z)
        return self.startup_pose_z

    def _gps_seed_yaw(self) -> float:
        if self.last_good_pose is not None:
            return self._yaw_from_pose(self.last_good_pose.pose.pose)
        return self.startup_pose_yaw

    def _sample_from_odom(self, msg: Odometry, now_sec: float) -> PoseSample:
        pose = msg.pose.pose
        return PoseSample(
            time_sec=now_sec,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            yaw=self._yaw_from_pose(pose),
            msg=msg,
            max_pose_covariance=self._odom_pose_max_covariance(msg),
        )

    def _odom_pose_max_covariance(self, msg: Odometry) -> Optional[float]:
        covariance = getattr(getattr(msg, "pose", None), "covariance", None)
        if covariance is None or len(covariance) < 36:
            return None

        values: List[float] = []
        for index in (0, 7, 14, 35):
            try:
                value = float(covariance[index])
            except (TypeError, ValueError):
                return None
            if not math.isfinite(value):
                return None
            if value < 0.0:
                return None
            values.append(value)

        # Many odometry publishers leave covariance as all zeros. Treat that as
        # unavailable instead of as perfect confidence.
        if not values or max(values) <= 0.0:
            return None
        return max(values)

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

    def _mark_lost(
        self,
        reasons: List[str],
        now_sec: Optional[float] = None,
    ) -> None:
        if now_sec is None:
            now_sec = self._now_sec()

        deduped = []
        for reason in reasons:
            if reason not in deduped:
                deduped.append(reason)

        if not self.lost_active:
            self.lost_active = True
            self.lost_since_sec = now_sec
            self.loss_event_id += 1
            self.recovery_attempts_this_event = 0
            self.localization_good_since = None
            self.recovery_promising_since = None
            self.logged_healthy = False
            self.current_loss_reasons = deduped
            self.get_logger().warn(
                "Localization suspected lost: " + "; ".join(deduped)
            )
            return

        if self.lost_since_sec is None:
            self.lost_since_sec = now_sec

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

    def _prune_pose_history(self, now_sec: float) -> None:
        horizon = max(
            self.pose_stale_timeout_sec,
            self.min_good_pose_age_sec,
            self.acceptance_pose_stability_window_sec,
            self.acceptance_confirmation_window_sec,
        ) + 5.0
        self.pose_history = [
            sample
            for sample in self.pose_history
            if now_sec - sample.time_sec <= horizon
        ]

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
            self.startup_xy_std,
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
        msg.pose.covariance = self._covariance_from_std(
            xy_std,
            yaw_std,
            xy_std,
        )
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
        z_std = xy_std if z_std is None else z_std
        z_var = z_std * z_std
        yaw_var = yaw_std * yaw_std
        covariance[0] = xy_var
        covariance[7] = xy_var
        covariance[14] = max(0.01, z_var)
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

    def _diag_first_float(
        self,
        fields: Dict[str, Any],
        names: List[str],
    ) -> Optional[float]:
        for name in names:
            value = self._diag_float(fields, name)
            if value is not None:
                return value
        return None

    def _diagnostic_level_value(
        self,
        fields: Dict[str, Any],
    ) -> Optional[float]:
        value = None
        for name in ("diagnostic_level", "level", "status_level"):
            value = self._diag_value(fields, name)
            if value is not None:
                break

        if value is None:
            status = self._diag_value(fields, "status")
            if isinstance(status, str):
                lowered = status.strip().lower()
                if lowered in ("ok", "healthy", "active", "nominal"):
                    return 0.0
                if lowered in ("warn", "warning", "degraded"):
                    return 1.0
                if lowered in ("error", "err", "bad", "failed", "inactive"):
                    return 2.0
                if lowered in ("stale", "timeout"):
                    return 3.0
            return None

        if isinstance(value, bool):
            return 0.0 if value else 2.0
        if isinstance(value, (float, int)):
            return float(value)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in ("ok", "healthy", "active", "nominal"):
                return 0.0
            if lowered in ("warn", "warning", "degraded"):
                return 1.0
            if lowered in ("error", "err", "bad", "failed", "inactive"):
                return 2.0
            if lowered in ("stale", "timeout"):
                return 3.0
            try:
                return float(lowered)
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

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

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
