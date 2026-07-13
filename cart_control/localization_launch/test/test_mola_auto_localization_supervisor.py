"""Offline tests for the MOLA auto-localization supervisor helpers."""

import importlib
import math
import sys
import types
from pathlib import Path


class FakeLogger:
    """Collect log calls without requiring rclpy."""

    def __init__(self):
        self.messages = []

    def debug(self, message):
        self.messages.append(("debug", message))

    def info(self, message):
        self.messages.append(("info", message))

    def warn(self, message):
        self.messages.append(("warn", message))

    def error(self, message):
        self.messages.append(("error", message))


class FakeStamp:
    """Minimal ROS stamp stand-in."""

    def __init__(self, stamp_sec=0.0):
        self.sec = int(stamp_sec)
        self.nanosec = int(round((stamp_sec - self.sec) * 1_000_000_000))


class FakeHeader:
    """Minimal ROS header stand-in."""

    def __init__(self, stamp_sec=0.0):
        self.stamp = FakeStamp(stamp_sec)
        self.frame_id = ""


class FakeClockNow:
    """Minimal clock now object."""

    def __init__(self, stamp_sec=0.0):
        self.nanoseconds = int(stamp_sec * 1_000_000_000)
        self._stamp_sec = stamp_sec

    def to_msg(self):
        return FakeStamp(self._stamp_sec)


class FakeClock:
    """Minimal ROS clock stand-in."""

    def __init__(self, stamp_sec=0.0):
        self.stamp_sec = stamp_sec

    def now(self):
        return FakeClockNow(self.stamp_sec)


class FakeNavSatStatus:
    """Minimal NavSatStatus stand-in."""

    def __init__(self, status=0):
        self.status = status


class FakeNavSatFix:
    """Minimal NavSatFix stand-in."""

    COVARIANCE_TYPE_UNKNOWN = 0
    COVARIANCE_TYPE_APPROXIMATED = 1

    def __init__(self):
        self.header = FakeHeader()
        self.status = FakeNavSatStatus()
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.position_covariance = [0.0] * 9
        self.position_covariance_type = self.COVARIANCE_TYPE_APPROXIMATED


class FakeQuaternion:
    """Minimal Quaternion stand-in."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class FakePosition:
    """Minimal Point stand-in."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class FakePose:
    """Minimal Pose stand-in."""

    def __init__(self):
        self.position = FakePosition()
        self.orientation = FakeQuaternion()


class FakePoseWithCovariance:
    """Minimal PoseWithCovariance stand-in."""

    def __init__(self):
        self.pose = FakePose()
        self.covariance = [0.0] * 36


class FakePoseWithCovarianceStamped:
    """Minimal PoseWithCovarianceStamped stand-in."""

    def __init__(self):
        self.header = FakeHeader()
        self.pose = FakePoseWithCovariance()


class FakePoseStamped:
    """Minimal PoseStamped stand-in."""

    def __init__(self):
        self.header = FakeHeader()
        self.pose = FakePose()


class FakeOdometry:
    """Minimal Odometry stand-in."""

    def __init__(self):
        self.header = FakeHeader()
        self.pose = FakePoseWithCovariance()


class FakePointCloud2:
    """Minimal PointCloud2 stand-in."""

    def __init__(self):
        self.width = 0
        self.height = 0


class FakeString:
    """Minimal std_msgs/String stand-in."""

    def __init__(self):
        self.data = ""


class FakeFuture:
    """Minimal async future stand-in."""

    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeNode:
    """Small rclpy.node.Node stand-in."""

    def __init__(self, *args, **kwargs):
        del args, kwargs


def install_ros_stubs():
    """Install just enough ROS modules to import the supervisor offline."""
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = lambda *args, **kwargs: None
    rclpy_mod.spin = lambda *args, **kwargs: None
    rclpy_mod.shutdown = lambda *args, **kwargs: None

    rclpy_node_mod = types.ModuleType("rclpy.node")
    rclpy_node_mod.Node = FakeNode

    rclpy_qos_mod = types.ModuleType("rclpy.qos")
    rclpy_qos_mod.qos_profile_sensor_data = object()

    geometry_mod = types.ModuleType("geometry_msgs")
    geometry_msg_mod = types.ModuleType("geometry_msgs.msg")
    geometry_msg_mod.Pose = FakePose
    geometry_msg_mod.PoseStamped = FakePoseStamped
    geometry_msg_mod.PoseWithCovarianceStamped = (
        FakePoseWithCovarianceStamped
    )
    geometry_msg_mod.Quaternion = FakeQuaternion

    nav_mod = types.ModuleType("nav_msgs")
    nav_msg_mod = types.ModuleType("nav_msgs.msg")
    nav_msg_mod.Odometry = FakeOdometry

    sensor_mod = types.ModuleType("sensor_msgs")
    sensor_msg_mod = types.ModuleType("sensor_msgs.msg")
    sensor_msg_mod.NavSatFix = FakeNavSatFix
    sensor_msg_mod.PointCloud2 = FakePointCloud2

    std_mod = types.ModuleType("std_msgs")
    std_msg_mod = types.ModuleType("std_msgs.msg")
    std_msg_mod.String = FakeString

    sys.modules.setdefault("rclpy", rclpy_mod)
    sys.modules.setdefault("rclpy.node", rclpy_node_mod)
    sys.modules.setdefault("rclpy.qos", rclpy_qos_mod)
    sys.modules.setdefault("geometry_msgs", geometry_mod)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msg_mod)
    sys.modules.setdefault("nav_msgs", nav_mod)
    sys.modules.setdefault("nav_msgs.msg", nav_msg_mod)
    sys.modules.setdefault("sensor_msgs", sensor_mod)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msg_mod)
    sys.modules.setdefault("std_msgs", std_mod)
    sys.modules.setdefault("std_msgs.msg", std_msg_mod)


def supervisor_module():
    """Import the supervisor module with ROS stubs installed."""
    install_ros_stubs()
    package_path = Path(__file__).parents[1]
    sys.path.insert(0, str(package_path))
    return importlib.import_module(
        "localization_launch.mola_auto_localization_supervisor"
    )


def make_supervisor(mod, now_sec=100.0):
    """Create an uninitialized supervisor with helper fields populated."""
    supervisor = object.__new__(mod.MolaAutoLocalizationSupervisor)
    logger = FakeLogger()
    supervisor._fake_logger = logger
    supervisor._now_sec = lambda: now_sec
    supervisor.get_logger = lambda: logger
    supervisor.get_clock = lambda: FakeClock(now_sec)
    supervisor._last_log_times = {}
    supervisor.map_frame = "map"
    supervisor.cloud_topic = "/velodyne_points"
    supervisor.mola_pose_topic = "/lidar_odometry/pose"
    supervisor.gps_min_status = 0
    supervisor.gps_fix_timeout_sec = 3.0
    supervisor.gps_stale_timeout_sec = 3.0
    supervisor.gps_seed_max_age_sec = 3.0
    supervisor.gps_max_covariance = 100.0
    supervisor.gps_reject_unknown_covariance = False
    supervisor.gps_seed_freeze_during_recovery = True
    supervisor.use_gps_relocalize = True
    supervisor.gps_topic = "/gps"
    supervisor.last_gps_msg_time = None
    supervisor.logged_first_gps_rx = False
    supervisor.gps_to_map_source = "landmark_calibration"
    supervisor.gps_calibration_config_path = ""
    supervisor.calibration_config_dir = ""
    supervisor.calibration_config_file = ""
    supervisor.gps_to_map_config_path = ""
    supervisor.gps_to_map_calibration = None
    supervisor.gps_to_map_method = "unloaded"
    supervisor.gps_recovery_xy_std = 10.0
    supervisor.gps_recovery_z_std = 5.0
    supervisor.gps_recovery_yaw_std = math.pi
    supervisor.gps_yaw_sweep_mode = "absolute"
    supervisor.gps_yaw_sweep_angles_deg = [
        0.0,
        45.0,
        90.0,
        135.0,
        180.0,
        225.0,
        270.0,
        315.0,
    ]
    supervisor.gps_yaw_sweep_max_attempts = 8
    supervisor.gps_yaw_sweep_candidate_yaw_std = 1.57
    supervisor.gps_yaw_sweep_accept_icp_quality = 0.0
    supervisor.gps_yaw_sweep_accept_dropped_frames_ratio = 0.2
    supervisor.yaw_candidate_settle_sec = 1.0
    supervisor.yaw_candidate_timeout_sec = 7.0
    supervisor.acceptance_near_seed_m = 6.0
    supervisor.acceptance_min_fresh_pose_count = 3
    supervisor.acceptance_require_diagnostics_ok = True
    supervisor.acceptance_require_match_quality = False
    supervisor.acceptance_allow_quality_fallback = False
    supervisor.acceptance_confirmation_window_sec = 2.0
    supervisor.acceptance_require_pose_stability = True
    supervisor.acceptance_pose_stability_window_sec = 2.0
    supervisor.acceptance_max_pose_drift_m = 1.0
    supervisor.acceptance_max_yaw_drift_rad = 0.35
    supervisor.acceptance_min_icp_quality = 0.75
    supervisor.acceptance_min_inlier_ratio = 0.60
    supervisor.acceptance_min_matched_points = 0
    supervisor.acceptance_max_icp_error = 0.75
    supervisor.acceptance_max_pose_covariance = 4.0
    supervisor.acceptance_max_dropped_frames = 0.10
    supervisor.acceptance_required_diagnostic_level = 0
    supervisor.last_good_pose = None
    supervisor.startup_pose_z = 0.0
    supervisor.startup_pose_yaw = 0.0
    supervisor.latest_gps_map_seed = None
    supervisor.last_gps_fix = None
    supervisor.gps_yaw_sweep_state = None
    supervisor.gps_yaw_sweep_failed_event_id = 0
    supervisor.loss_event_id = 1
    supervisor.recovery_episode_id = 0
    supervisor.lost_active = False
    supervisor.lost_since_sec = None
    supervisor.current_loss_reasons = []
    supervisor.recovery_attempts_this_event = 0
    supervisor.max_relocalize_attempts_per_event = 3
    supervisor.enable_recovery_relocalize = True
    supervisor.gps_yaw_sweep_enabled = False
    supervisor.pending_relocalize_future = None
    supervisor.pending_relocalize_start = None
    supervisor.pending_relocalize_context = ""
    supervisor.last_relocalize_time = None
    supervisor.gps_relocalize_cooldown_sec = 8.0
    supervisor.relocalize_cooldown_sec = 8.0
    supervisor.pose_sequence = 0
    supervisor.pose_history = []
    supervisor.last_pose_time = None
    supervisor.last_pose_sample = None
    supervisor.cloud_stale_timeout_sec = 2.0
    supervisor.pose_stale_timeout_sec = 3.0
    supervisor.last_diag_time = None
    supervisor.last_diag_fields = {}
    supervisor.bad_diag_since = None
    supervisor.bad_diag_reasons = []
    supervisor.localization_good_since = None
    supervisor.logged_healthy = False
    supervisor.recovery_xy_std = 6.0
    supervisor.recovery_yaw_std = math.pi
    supervisor.manual_pose_topic = "/initialpose"
    supervisor.manual_pose_cooldown_sec = 30.0
    supervisor.manual_pose_cooldown_until = 0.0
    supervisor.manual_pose_lockout_sec = 30.0
    supervisor.manual_pose_lockout_until = 0.0
    supervisor.manual_pose_pending = False
    supervisor.manual_pose_time = None
    supervisor.manual_pose_promising_since = None
    supervisor.manual_pose_count = 0
    supervisor.post_recovery_lockout_sec = 25.0
    supervisor.post_recovery_lockout_until = 0.0
    supervisor.post_recovery_lockout_reason = ""
    supervisor.recovery_require_sustained_loss = True
    supervisor.recovery_min_lost_duration_sec = 3.0
    supervisor.startup_relocalize_complete = False
    supervisor.recovery_promising_since = None
    supervisor._ensure_relocalize_client = lambda: None
    return supervisor


def make_fix(mod, stamp_sec=100.0, status=0, covariance=1.0):
    """Build a fake GPS fix."""
    fix = mod.NavSatFix()
    fix.header = FakeHeader(stamp_sec)
    fix.status = FakeNavSatStatus(status)
    fix.latitude = 40.0
    fix.longitude = -83.0
    fix.altitude = 250.0
    fix.position_covariance = [0.0] * 9
    fix.position_covariance[0] = covariance
    fix.position_covariance[4] = covariance
    fix.position_covariance[8] = covariance
    fix.position_covariance_type = fix.COVARIANCE_TYPE_APPROXIMATED
    return fix


def make_seed(mod, x=10.0, y=20.0, time_sec=99.0):
    """Build a cached GPS map seed."""
    return mod.GpsMapSeedSample(
        time_sec=time_sec,
        fix_time_sec=time_sec,
        latitude=40.0,
        longitude=-83.0,
        map_x=x,
        map_y=y,
        z=0.0,
        yaw=0.0,
        yaw_source="startup_pose",
        status=0,
        max_covariance=1.0,
    )


def set_pose_sample(
    mod,
    supervisor,
    now_sec,
    x=10.0,
    y=20.0,
    yaw=0.0,
    covariance=None,
):
    """Install a fresh MOLA pose sample on the supervisor."""
    odom = FakeOdometry()
    odom.header = FakeHeader(now_sec)
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation = supervisor._quaternion_from_yaw(yaw)
    if covariance is not None:
        odom.pose.covariance[0] = covariance
        odom.pose.covariance[7] = covariance
        odom.pose.covariance[14] = covariance
        odom.pose.covariance[35] = covariance
    supervisor.pose_sequence += 1
    supervisor.last_pose_time = now_sec
    supervisor.last_pose_sample = mod.PoseSample(
        time_sec=now_sec,
        x=x,
        y=y,
        z=0.0,
        yaw=yaw,
        msg=odom,
        max_pose_covariance=(
            covariance if covariance is not None and covariance > 0.0 else None
        ),
    )
    supervisor.pose_history.append(supervisor.last_pose_sample)


def set_diagnostics(
    supervisor,
    now_sec,
    active=True,
    icp_quality=0.8,
    inlier_ratio=0.8,
    matched_points=None,
    icp_error=0.2,
    dropped_frames_ratio=0.0,
    diagnostic_level=0,
    bad_since=None,
    include_match_quality=True,
):
    """Install a MOLA diagnostics sample on the supervisor."""
    supervisor.last_diag_time = now_sec
    supervisor.last_diag_fields = {
        "active": active,
        "dropped_frames_ratio": dropped_frames_ratio,
        "diagnostic_level": diagnostic_level,
    }
    if include_match_quality:
        if icp_quality is not None:
            supervisor.last_diag_fields["icp_quality"] = icp_quality
        if inlier_ratio is not None:
            supervisor.last_diag_fields["inlier_ratio"] = inlier_ratio
        if matched_points is not None:
            supervisor.last_diag_fields["matched_points"] = matched_points
        if icp_error is not None:
            supervisor.last_diag_fields["icp_error"] = icp_error
    supervisor.bad_diag_since = bad_since
    supervisor.bad_diag_reasons = (
        ["test diagnostics bad"] if bad_since is not None else []
    )


def make_active_yaw_state(mod, angles=None):
    """Build a yaw sweep state with one candidate already requested."""
    return mod.GpsYawSweepState(
        map_x=10.0,
        map_y=20.0,
        z=0.0,
        seed_time_sec=99.0,
        seed_fix_time_sec=99.0,
        seed_source="GPS map seed",
        yaw_mode="absolute",
        base_yaw_rad=0.0,
        angles_deg=angles if angles is not None else [0.0, 90.0],
        next_index=1,
        attempts=1,
        active_yaw_deg=0.0,
        active_yaw_rad=0.0,
        candidate_sent_time=100.0,
        candidate_pose_sequence=0,
        candidate_service_success=True,
    )


def test_gps_fix_rejects_stale_stamp_and_bad_covariance():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)

    stale = make_fix(mod, stamp_sec=90.0)
    reason = supervisor._gps_fix_reject_reason(stale, now_sec=100.0)
    assert "stale" in reason

    high_covariance = make_fix(mod, stamp_sec=100.0, covariance=101.0)
    reason = supervisor._gps_fix_reject_reason(
        high_covariance,
        now_sec=100.0,
    )
    assert "covariance" in reason
    assert "exceeds" in reason


def test_gps_callback_updates_cached_map_seed():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    supervisor.gps_topic = "/fix_valid"
    supervisor._gps_to_map_xy = lambda fix: (12.5, -3.0)

    supervisor._gps_callback(make_fix(mod, stamp_sec=100.0))

    seed = supervisor.latest_gps_map_seed
    assert seed is not None
    assert seed.map_x == 12.5
    assert seed.map_y == -3.0
    assert seed.latitude == 40.0
    assert seed.longitude == -83.0
    assert seed.yaw_source == "startup_pose"
    assert supervisor.last_gps_msg_time == 100.0
    assert any(
        "GPS_RX topic=/fix_valid" in message
        for _level, message in supervisor._fake_logger.messages
    )


def test_gps_waiting_logs_when_enabled_and_no_messages():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    supervisor.gps_topic = "/fix_valid"
    supervisor.last_cloud_time = 100.0

    supervisor._log_waiting_states(100.0)

    assert any(
        "GPS_WAITING topic=/fix_valid no messages received yet" in message
        for _level, message in supervisor._fake_logger.messages
    )


def test_gps_conversion_log_includes_calibration_details():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    supervisor.gps_topic = "/fix_valid"
    supervisor.gps_to_map_config_path = "/maps/with_gps2_adjusted_route.yaml"
    supervisor.gps_to_map_method = "navigation.simple_gps_util.gps_to_local"
    supervisor.gps_to_map_calibration = (
        38.433,
        -78.862,
        45.0,
        90.0,
        1.5,
        -2.5,
        12.25,
    )
    supervisor._gps_to_map_xy = lambda fix: (12.5, -3.0)

    supervisor._gps_callback(make_fix(mod, stamp_sec=100.0, covariance=0.25))

    conversion_logs = [
        message
        for _level, message in supervisor._fake_logger.messages
        if "GPS_CONVERSION" in message
    ]
    assert conversion_logs
    log = conversion_logs[-1]
    assert "topic=/fix_valid" in log
    assert "lat=40.000000000000" in log
    assert "lon=-83.000000000000" in log
    assert "cov_x=0.250000" in log
    assert "map_x=12.500" in log
    assert "map_y=-3.000" in log
    assert "accepted=true" in log
    assert "method=navigation.simple_gps_util.gps_to_local" in log
    assert "config=/maps/with_gps2_adjusted_route.yaml" in log
    assert "rotation_deg=12.250000000" in log


def test_gps_seed_is_frozen_for_yaw_sweep_episode():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    supervisor.last_gps_fix = mod.GpsFixSample(
        time_sec=99.0,
        stamp_sec=99.0,
        latitude=40.0,
        longitude=-83.0,
        altitude=250.0,
        status=0,
        max_covariance=1.0,
    )
    supervisor.latest_gps_map_seed = make_seed(mod, x=10.0, y=20.0)
    supervisor.lost_active = True
    supervisor._cloud_ready = lambda now: True
    supervisor._request_relocalize_with_pose = lambda *args, **kwargs: True

    assert supervisor._start_gps_yaw_sweep(100.0) is True
    state = supervisor.gps_yaw_sweep_state
    assert state.map_x == 10.0
    assert state.map_y == 20.0

    supervisor._gps_to_map_xy = lambda fix: (100.0, 200.0)
    supervisor._gps_callback(make_fix(mod, stamp_sec=100.0))

    assert supervisor.latest_gps_map_seed.map_x == 100.0
    assert supervisor.gps_yaw_sweep_state.map_x == 10.0
    assert supervisor.gps_yaw_sweep_state.map_y == 20.0


def test_yaw_candidates_reuse_same_frozen_seed_xy():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    poses = []
    supervisor._cloud_ready = lambda now: True

    def capture_request(kind, reason, pose_msg, *args, **kwargs):
        del kind, reason, args, kwargs
        pose = pose_msg.pose.pose
        poses.append((pose.position.x, pose.position.y))
        return True

    supervisor._request_relocalize_with_pose = capture_request
    supervisor.gps_yaw_sweep_state = mod.GpsYawSweepState(
        map_x=12.0,
        map_y=-4.0,
        z=0.0,
        seed_time_sec=99.0,
        seed_fix_time_sec=99.0,
        seed_source="GPS map seed",
        yaw_mode="absolute",
        base_yaw_rad=0.0,
        angles_deg=[0.0, 90.0],
    )

    supervisor._try_next_gps_yaw_candidate(100.0)
    supervisor.gps_yaw_sweep_state.candidate_sent_time = None
    supervisor._try_next_gps_yaw_candidate(101.0)

    assert poses == [(12.0, -4.0), (12.0, -4.0)]


def test_yaw_sweep_stops_immediately_after_candidate_acceptance():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=102.0)
    supervisor.acceptance_confirmation_window_sec = 0.0
    supervisor.lost_active = True
    supervisor.gps_yaw_sweep_state = make_active_yaw_state(mod)
    set_pose_sample(mod, supervisor, now_sec=100.1, x=10.1, y=20.0)
    set_pose_sample(mod, supervisor, now_sec=101.1, x=10.2, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=102.1, x=10.2, y=20.1)
    set_diagnostics(supervisor, now_sec=102.1)

    supervisor._process_gps_yaw_sweep(102.1)

    assert supervisor.gps_yaw_sweep_state is None
    assert supervisor.lost_active is False
    assert supervisor.last_good_pose is not None
    assert supervisor.post_recovery_lockout_until > 102.1


def test_healthy_localization_blocks_relocalization_paths():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    set_pose_sample(mod, supervisor, now_sec=99.5)
    set_diagnostics(supervisor, now_sec=99.5)
    supervisor.lost_active = False
    supervisor.enable_startup_relocalize = True
    supervisor._ensure_relocalize_client = (
        lambda: (_ for _ in ()).throw(AssertionError("service lookup"))
    )
    supervisor._cloud_ready = lambda now: True

    supervisor._maybe_startup_relocalize(100.0)
    supervisor._maybe_recovery_relocalize(100.0)
    requested = supervisor._request_relocalize_with_pose(
        "recovery",
        "test",
        pose_msg=FakePoseWithCovarianceStamped(),
        pose_source="GPS seed",
        xy_std=1.0,
        z_std=1.0,
        yaw_std=1.0,
    )
    started = supervisor._start_gps_yaw_sweep(100.0)

    assert requested is False
    assert started is False
    assert supervisor.gps_yaw_sweep_state is None


def test_recovery_waits_for_sustained_loss_before_request():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=102.0)
    supervisor.lost_active = True
    supervisor.lost_since_sec = 100.0
    supervisor.use_gps_relocalize = False
    supervisor._cloud_ready = lambda now: True
    calls = []
    supervisor._request_relocalize = (
        lambda *args, **kwargs: calls.append((args, kwargs)) or True
    )

    supervisor._maybe_recovery_relocalize(102.9)
    assert calls == []

    supervisor._maybe_recovery_relocalize(103.1)
    assert len(calls) == 1


def test_healthy_pose_cancels_active_gps_recovery_activity():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    set_pose_sample(mod, supervisor, now_sec=99.8)
    set_diagnostics(supervisor, now_sec=99.8)
    supervisor.lost_active = False
    supervisor.gps_yaw_sweep_state = make_active_yaw_state(mod)
    future = FakeFuture()
    supervisor.pending_relocalize_future = future
    supervisor.pending_relocalize_context = "gps_yaw_sweep"
    supervisor.pending_relocalize_start = 99.5

    cancelled = supervisor._cancel_recovery_activity_if_healthy(100.0)

    assert cancelled is True
    assert supervisor.gps_yaw_sweep_state is None
    assert supervisor.pending_relocalize_future is None
    assert future.cancelled is True
    assert any(
        "RECOVERY_SUPPRESSED_HEALTHY" in message
        for _level, message in supervisor._fake_logger.messages
    )


def test_fresh_pose_without_required_diagnostics_does_not_accept():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=102.0)
    supervisor.lost_active = True
    supervisor._cloud_ready = lambda now: True
    calls = []
    supervisor._request_relocalize_with_pose = (
        lambda *args, **kwargs: calls.append((args, kwargs)) or True
    )
    supervisor.gps_yaw_sweep_state = make_active_yaw_state(mod)
    set_pose_sample(mod, supervisor, now_sec=100.2, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=100.9, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=101.5, x=10.2, y=20.2)

    supervisor._process_gps_yaw_sweep(102.0)

    assert supervisor.gps_yaw_sweep_state is not None
    assert supervisor.gps_yaw_sweep_state.active_yaw_deg == 0.0
    assert supervisor.post_recovery_lockout_until == 0.0
    assert calls == []

    supervisor._process_gps_yaw_sweep(108.1)

    assert supervisor.gps_yaw_sweep_state is not None
    assert supervisor.gps_yaw_sweep_state.active_yaw_deg == 90.0
    assert supervisor.post_recovery_lockout_until == 0.0
    assert len(calls) == 1


def test_bad_diagnostics_reject_candidate_without_lockout():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=102.0)
    supervisor.lost_active = True
    supervisor._cloud_ready = lambda now: True
    calls = []
    supervisor._request_relocalize_with_pose = (
        lambda *args, **kwargs: calls.append((args, kwargs)) or True
    )
    supervisor.gps_yaw_sweep_state = make_active_yaw_state(mod)
    set_pose_sample(mod, supervisor, now_sec=100.1, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=101.1, x=10.2, y=20.2)
    set_pose_sample(mod, supervisor, now_sec=102.1, x=10.2, y=20.2)
    set_diagnostics(
        supervisor,
        now_sec=101.5,
        active=False,
        icp_quality=0.1,
        bad_since=101.5,
    )

    supervisor._process_gps_yaw_sweep(102.1)

    assert supervisor.gps_yaw_sweep_state is not None
    assert supervisor.gps_yaw_sweep_state.active_yaw_deg == 90.0
    assert supervisor.post_recovery_lockout_until == 0.0
    assert len(calls) == 1


def test_bad_match_quality_rejects_candidate_without_lockout():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=102.0)
    supervisor.acceptance_require_match_quality = True
    supervisor.lost_active = True
    supervisor._cloud_ready = lambda now: True
    calls = []
    supervisor._request_relocalize_with_pose = (
        lambda *args, **kwargs: calls.append((args, kwargs)) or True
    )
    supervisor.gps_yaw_sweep_state = make_active_yaw_state(mod)
    set_pose_sample(mod, supervisor, now_sec=100.1, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=101.1, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=102.1, x=10.1, y=20.1)
    set_diagnostics(
        supervisor,
        now_sec=102.1,
        icp_quality=0.70,
        inlier_ratio=0.8,
        icp_error=0.2,
    )

    supervisor._process_gps_yaw_sweep(102.1)

    assert supervisor.gps_yaw_sweep_state is not None
    assert supervisor.gps_yaw_sweep_state.active_yaw_deg == 90.0
    assert supervisor.post_recovery_lockout_until == 0.0
    assert len(calls) == 1


def test_no_match_quality_rejects_unless_fallback_enabled():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=102.0)
    supervisor.acceptance_require_match_quality = True
    supervisor.lost_active = True
    supervisor._cloud_ready = lambda now: True
    calls = []
    supervisor._request_relocalize_with_pose = (
        lambda *args, **kwargs: calls.append((args, kwargs)) or True
    )
    supervisor.gps_yaw_sweep_state = make_active_yaw_state(mod)
    set_pose_sample(mod, supervisor, now_sec=100.1, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=101.1, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=102.1, x=10.1, y=20.1)
    set_diagnostics(
        supervisor,
        now_sec=102.1,
        include_match_quality=False,
    )

    supervisor._process_gps_yaw_sweep(102.1)

    assert supervisor.gps_yaw_sweep_state is not None
    assert supervisor.gps_yaw_sweep_state.active_yaw_deg == 0.0
    assert supervisor.post_recovery_lockout_until == 0.0
    assert calls == []

    supervisor._process_gps_yaw_sweep(108.1)

    assert supervisor.gps_yaw_sweep_state is not None
    assert supervisor.gps_yaw_sweep_state.active_yaw_deg == 90.0
    assert supervisor.post_recovery_lockout_until == 0.0
    assert len(calls) == 1


def test_pose_instability_rejects_candidate_without_lockout():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=102.0)
    supervisor.lost_active = True
    supervisor._cloud_ready = lambda now: True
    calls = []
    supervisor._request_relocalize_with_pose = (
        lambda *args, **kwargs: calls.append((args, kwargs)) or True
    )
    supervisor.gps_yaw_sweep_state = make_active_yaw_state(mod)
    set_pose_sample(mod, supervisor, now_sec=100.1, x=10.0, y=20.0)
    set_pose_sample(mod, supervisor, now_sec=101.1, x=11.4, y=20.0)
    set_pose_sample(mod, supervisor, now_sec=102.1, x=11.5, y=20.0)
    set_diagnostics(supervisor, now_sec=102.1)

    supervisor._process_gps_yaw_sweep(102.1)

    assert supervisor.gps_yaw_sweep_state is not None
    assert supervisor.gps_yaw_sweep_state.active_yaw_deg == 90.0
    assert supervisor.post_recovery_lockout_until == 0.0
    assert len(calls) == 1


def test_explicit_quality_fallback_can_accept_without_direct_match_metric():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=102.0)
    supervisor.acceptance_confirmation_window_sec = 0.0
    supervisor.acceptance_require_diagnostics_ok = False
    supervisor.acceptance_require_match_quality = True
    supervisor.acceptance_allow_quality_fallback = True
    supervisor.lost_active = True
    supervisor.gps_yaw_sweep_state = make_active_yaw_state(mod)
    set_pose_sample(mod, supervisor, now_sec=100.1, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=101.1, x=10.2, y=20.2)
    set_pose_sample(mod, supervisor, now_sec=102.1, x=10.2, y=20.2)
    set_diagnostics(
        supervisor,
        now_sec=102.1,
        include_match_quality=False,
    )

    supervisor._process_gps_yaw_sweep(102.1)

    assert supervisor.gps_yaw_sweep_state is None
    assert supervisor.lost_active is False
    assert supervisor.post_recovery_lockout_until > 102.1


def test_promising_candidate_waits_for_confirmation_before_lockout():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=102.0)
    supervisor.lost_active = True
    supervisor.gps_yaw_sweep_state = make_active_yaw_state(mod)
    set_pose_sample(mod, supervisor, now_sec=100.1, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=101.1, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=102.1, x=10.1, y=20.1)
    set_diagnostics(supervisor, now_sec=102.1)

    supervisor._process_gps_yaw_sweep(102.1)

    assert supervisor.gps_yaw_sweep_state is not None
    assert supervisor.gps_yaw_sweep_state.active_yaw_deg == 0.0
    assert supervisor.post_recovery_lockout_until == 0.0

    set_pose_sample(mod, supervisor, now_sec=102.3, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=103.3, x=10.1, y=20.1)
    set_pose_sample(mod, supervisor, now_sec=104.3, x=10.1, y=20.1)
    set_diagnostics(supervisor, now_sec=104.3)

    supervisor._process_gps_yaw_sweep(104.3)

    assert supervisor.gps_yaw_sweep_state is None
    assert supervisor.lost_active is False
    assert supervisor.post_recovery_lockout_until > 104.3


def test_post_recovery_lockout_blocks_relocalization_calls():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    supervisor.lost_active = True
    supervisor.post_recovery_lockout_until = 120.0
    calls = []
    supervisor._cloud_ready = lambda now: True
    supervisor._request_relocalize = (
        lambda *args, **kwargs: calls.append((args, kwargs)) or True
    )

    supervisor._maybe_recovery_relocalize(100.0)

    assert calls == []
    assert supervisor.recovery_attempts_this_event == 0


def test_gps_recovery_seed_requires_fresh_cached_map_seed():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    supervisor.last_gps_fix = mod.GpsFixSample(
        time_sec=99.0,
        stamp_sec=99.0,
        latitude=40.0,
        longitude=-83.0,
        altitude=250.0,
        status=0,
        max_covariance=1.0,
    )

    assert "map seed" in supervisor._gps_unavailable_reason(100.0)

    supervisor.latest_gps_map_seed = make_seed(mod, time_sec=95.0)
    assert "stale" in supervisor._gps_unavailable_reason(100.0)

    supervisor.latest_gps_map_seed.time_sec = 99.0
    seed, reason = supervisor._gps_recovery_seed(100.0)
    assert reason == ""
    assert seed == (10.0, 20.0, 0.0, 1.0)


def test_recovery_request_is_gated_by_health_and_manual_cooldown():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    calls = []
    supervisor.enable_recovery_relocalize = True
    supervisor.lost_active = False

    supervisor._maybe_recovery_relocalize(100.0)
    assert calls == []

    supervisor.lost_active = True
    supervisor.manual_pose_cooldown_until = 110.0
    supervisor._cancel_gps_yaw_sweep = lambda reason: False
    supervisor._maybe_recovery_relocalize(100.0)
    assert calls == []

    supervisor.manual_pose_cooldown_until = 0.0
    supervisor.lost_since_sec = 96.0
    supervisor.recovery_attempts_this_event = 0
    supervisor.max_relocalize_attempts_per_event = 3
    supervisor._cloud_ready = lambda now: True
    supervisor.gps_yaw_sweep_enabled = False
    supervisor._request_relocalize = (
        lambda *args, **kwargs: calls.append((args, kwargs)) or True
    )

    supervisor._maybe_recovery_relocalize(100.0)
    assert len(calls) == 1
    assert supervisor.recovery_attempts_this_event == 1


def test_manual_pose_followed_by_healthy_pose_enters_lockout():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    supervisor.acceptance_confirmation_window_sec = 0.0

    supervisor._manual_pose_callback(FakePoseWithCovarianceStamped())
    set_pose_sample(mod, supervisor, now_sec=100.1, x=2.0, y=3.0)
    set_pose_sample(mod, supervisor, now_sec=101.1, x=2.1, y=3.0)
    set_pose_sample(mod, supervisor, now_sec=102.1, x=2.1, y=3.0)
    set_diagnostics(supervisor, now_sec=102.1)

    supervisor._update_healthy_state(102.1)

    assert supervisor.manual_pose_pending is False
    assert supervisor.manual_pose_lockout_until > 102.1
    assert supervisor.last_good_pose is not None


def test_manual_pose_fresh_pose_without_diagnostics_stays_pending():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)

    supervisor._manual_pose_callback(FakePoseWithCovarianceStamped())
    set_pose_sample(mod, supervisor, now_sec=101.0, x=2.0, y=3.0)

    supervisor._update_healthy_state(101.0)

    assert supervisor.manual_pose_pending is True
    assert supervisor.manual_pose_lockout_until == 0.0
    assert supervisor.last_good_pose is None


def test_manual_pose_cooldown_blocks_request_before_service_lookup():
    mod = supervisor_module()
    supervisor = make_supervisor(mod, now_sec=100.0)
    supervisor.manual_pose_cooldown_until = 105.0

    requested = supervisor._request_relocalize_with_pose(
        "recovery",
        "test",
        pose_msg=None,
        pose_source="GPS seed",
        xy_std=1.0,
        z_std=1.0,
        yaw_std=1.0,
    )

    assert requested is False


def test_pending_relocalization_prevents_overlapping_yaw_candidate():
    mod = supervisor_module()
    supervisor = make_supervisor(mod)
    calls = []
    supervisor.pending_relocalize_future = object()
    supervisor.gps_yaw_sweep_state = mod.GpsYawSweepState(
        map_x=10.0,
        map_y=20.0,
        z=0.0,
        seed_time_sec=99.0,
        seed_fix_time_sec=99.0,
        seed_source="GPS map seed",
        yaw_mode="absolute",
        base_yaw_rad=0.0,
        angles_deg=[0.0],
    )
    supervisor._request_relocalize_with_pose = (
        lambda *args, **kwargs: calls.append((args, kwargs)) or True
    )

    supervisor._process_gps_yaw_sweep(100.0)

    assert calls == []


def test_new_yaw_candidate_param_can_override_legacy_default():
    mod = supervisor_module()
    supervisor = make_supervisor(mod)
    default = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
    values = {
        "gps_yaw_sweep_angles_deg": default,
        "gps_yaw_candidates_deg": [10.0, 20.0, 10.0],
    }
    supervisor.get_parameter = lambda name: types.SimpleNamespace(
        value=values[name]
    )

    assert supervisor._gps_yaw_sweep_angles_from_param() == [10.0, 20.0]


def test_yaw_candidate_limit_uses_configured_max_attempts():
    mod = supervisor_module()
    supervisor = make_supervisor(mod)
    supervisor.gps_yaw_sweep_angles_deg = [
        0.0,
        45.0,
        90.0,
        135.0,
    ]
    supervisor.gps_yaw_sweep_max_attempts = 2

    assert supervisor._gps_yaw_sweep_candidate_angles() == [0.0, 45.0]


def test_quaternion_yaw_round_trip_works_with_stubs():
    mod = supervisor_module()
    supervisor = make_supervisor(mod)

    pose = FakePose()
    pose.orientation = supervisor._quaternion_from_yaw(math.radians(90.0))

    assert math.isclose(
        supervisor._yaw_from_pose(pose),
        math.radians(90.0),
        abs_tol=1e-6,
    )


def test_usb_gps_launch_wiring_is_conditional_and_topic_is_forwarded():
    repo_root = Path(__file__).parents[3]
    cart_launch = (
        repo_root
        / "cart_control"
        / "cart_launch"
        / "launch"
        / "mola_autonomy.launch.py"
    ).read_text(encoding="utf-8")
    wrapper_launch = (
        repo_root
        / "cart_control"
        / "localization_launch"
        / "launch"
        / "mola_autonomy.launch.py"
    ).read_text(encoding="utf-8")

    assert "enable_usb_gps" in cart_launch
    assert "gps_port" in cart_launch
    assert "gps_baudrate" in cart_launch
    assert "gps_raw_topic" in cart_launch
    assert "gps_valid_topic" in cart_launch
    assert "garmin_gps18x_valid.launch.py" in cart_launch
    assert "condition=IfCondition(enable_usb_gps)" in cart_launch
    assert '"port": gps_port' in cart_launch
    assert '"baud": gps_baudrate' in cart_launch
    assert '"input_topic": gps_raw_topic' in cart_launch
    assert '"output_topic": gps_valid_topic' in cart_launch
    assert '"gps_topic": effective_gps_topic' in cart_launch
    assert "USB_GPS_CONFIG enabled=" in cart_launch

    assert "enable_usb_gps" in wrapper_launch
    assert '"gps_topic": LaunchConfiguration("gps_topic")' in wrapper_launch


def test_launch_script_cleanup_covers_usb_gps_processes():
    repo_root = Path(__file__).parents[3]
    script = (repo_root / "scripts" / "launch_mola_stack.sh").read_text(
        encoding="utf-8"
    )

    assert "LAUNCH_CLEANUP starting" in script
    assert "LAUNCH_CLEANUP sent SIGINT" in script
    assert "LAUNCH_CLEANUP complete" in script
    assert "garmin_gps18x_driver|valid_fix_filter" in script
