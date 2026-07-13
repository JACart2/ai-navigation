import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _source_or_share(source_path, package_name, *share_parts):
    if os.path.exists(source_path):
        return source_path
    return os.path.join(get_package_share_directory(package_name), *share_parts)


def generate_launch_description():
    cart_launch_path = os.path.join(
        get_package_share_directory("cart_launch"),
        "launch",
        "mola_autonomy.launch.py",
    )
    navigation_maps_dir = _source_or_share(
        "/root/dev_ws/src/ai-navigation/navigation/maps",
        "navigation",
        "maps",
    )
    default_rviz_config = _source_or_share(
        (
            "/root/dev_ws/src/ai-navigation/cart_control/cart_launch/rviz/"
            "mola_localization.rviz"
        ),
        "cart_launch",
        "rviz",
        "mola_localization.rviz",
    )

    forwarded_args = {
        "cart": LaunchConfiguration("cart"),
        "cart_name": LaunchConfiguration("cart_name"),
        "cart_config_path": LaunchConfiguration("cart_config_path"),
        "rviz_config": LaunchConfiguration("rviz_config"),
        "launch_rviz": LaunchConfiguration("use_rviz"),
        "start_velodyne": LaunchConfiguration("start_velodyne"),
        "navigation_start_delay": LaunchConfiguration("navigation_start_delay"),
        "enable_motor": LaunchConfiguration("enable_motor"),
        "enable_mola_auto_localization": LaunchConfiguration(
            "enable_mola_auto_localization"
        ),
        "enable_usb_gps": LaunchConfiguration("enable_usb_gps"),
        "gps_port": LaunchConfiguration("gps_port"),
        "gps_baudrate": LaunchConfiguration("gps_baudrate"),
        "gps_raw_topic": LaunchConfiguration("gps_raw_topic"),
        "gps_valid_topic": LaunchConfiguration("gps_valid_topic"),
        "use_gps_relocalize": LaunchConfiguration("use_gps_relocalize"),
        "gps_topic": LaunchConfiguration("gps_topic"),
        "manual_pose_cooldown_sec": LaunchConfiguration(
            "manual_pose_cooldown_sec"
        ),
        "manual_pose_lockout_sec": LaunchConfiguration(
            "manual_pose_lockout_sec"
        ),
        "gps_fix_timeout_sec": LaunchConfiguration("gps_fix_timeout_sec"),
        "gps_seed_max_age_sec": LaunchConfiguration("gps_seed_max_age_sec"),
        "gps_max_covariance": LaunchConfiguration("gps_max_covariance"),
        "gps_relocalize_cooldown_sec": LaunchConfiguration(
            "gps_relocalize_cooldown_sec"
        ),
        "gps_yaw_sweep_enabled": LaunchConfiguration(
            "gps_yaw_sweep_enabled"
        ),
        "gps_yaw_sweep_mode": LaunchConfiguration("gps_yaw_sweep_mode"),
        "gps_yaw_sweep_accept_icp_quality": LaunchConfiguration(
            "gps_yaw_sweep_accept_icp_quality"
        ),
        "yaw_candidate_settle_sec": LaunchConfiguration(
            "yaw_candidate_settle_sec"
        ),
        "yaw_candidate_timeout_sec": LaunchConfiguration(
            "yaw_candidate_timeout_sec"
        ),
        "post_recovery_lockout_sec": LaunchConfiguration(
            "post_recovery_lockout_sec"
        ),
        "acceptance_near_seed_m": LaunchConfiguration("acceptance_near_seed_m"),
        "acceptance_min_fresh_pose_count": LaunchConfiguration(
            "acceptance_min_fresh_pose_count"
        ),
        "acceptance_require_diagnostics_ok": LaunchConfiguration(
            "acceptance_require_diagnostics_ok"
        ),
        "acceptance_require_match_quality": LaunchConfiguration(
            "acceptance_require_match_quality"
        ),
        "acceptance_allow_quality_fallback": LaunchConfiguration(
            "acceptance_allow_quality_fallback"
        ),
        "acceptance_confirmation_window_sec": LaunchConfiguration(
            "acceptance_confirmation_window_sec"
        ),
        "acceptance_require_pose_stability": LaunchConfiguration(
            "acceptance_require_pose_stability"
        ),
        "acceptance_pose_stability_window_sec": LaunchConfiguration(
            "acceptance_pose_stability_window_sec"
        ),
        "acceptance_max_pose_drift_m": LaunchConfiguration(
            "acceptance_max_pose_drift_m"
        ),
        "acceptance_max_yaw_drift_rad": LaunchConfiguration(
            "acceptance_max_yaw_drift_rad"
        ),
        "acceptance_min_icp_quality": LaunchConfiguration(
            "acceptance_min_icp_quality"
        ),
        "acceptance_min_inlier_ratio": LaunchConfiguration(
            "acceptance_min_inlier_ratio"
        ),
        "acceptance_min_matched_points": LaunchConfiguration(
            "acceptance_min_matched_points"
        ),
        "acceptance_max_icp_error": LaunchConfiguration(
            "acceptance_max_icp_error"
        ),
        "acceptance_max_pose_covariance": LaunchConfiguration(
            "acceptance_max_pose_covariance"
        ),
        "acceptance_max_dropped_frames": LaunchConfiguration(
            "acceptance_max_dropped_frames"
        ),
        "acceptance_required_diagnostic_level": LaunchConfiguration(
            "acceptance_required_diagnostic_level"
        ),
        "recovery_require_sustained_loss": LaunchConfiguration(
            "recovery_require_sustained_loss"
        ),
        "recovery_min_lost_duration_sec": LaunchConfiguration(
            "recovery_min_lost_duration_sec"
        ),
        "motor_port": LaunchConfiguration("motor_port"),
        "motor_baudrate": LaunchConfiguration("motor_baudrate"),
        "enable_aad": LaunchConfiguration("enable_aad"),
        "graph_dir": LaunchConfiguration("graph_dir"),
        "graph_file": LaunchConfiguration("graph_file"),
        "graph_coordinate_format": LaunchConfiguration("graph_coordinate_format"),
        "calibration_config_dir": LaunchConfiguration("calibration_config_dir"),
        "calibration_config_file": LaunchConfiguration("calibration_config_file"),
        "map_file": LaunchConfiguration("map_file"),
        "lidar_topic": LaunchConfiguration("lidar_topic"),
        "base_frame": LaunchConfiguration("base_frame"),
        "lidar_frame": LaunchConfiguration("lidar_frame"),
        "lidar_x": LaunchConfiguration("lidar_x"),
        "lidar_y": LaunchConfiguration("lidar_y"),
        "lidar_z": LaunchConfiguration("lidar_z"),
        "lidar_yaw": LaunchConfiguration("lidar_yaw"),
        "lidar_pitch": LaunchConfiguration("lidar_pitch"),
        "lidar_roll": LaunchConfiguration("lidar_roll"),
        "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart",
                default_value="madison",
                description=(
                    "Compatibility cart selector. Prefer cart_name for new "
                    "commands."
                ),
            ),
            DeclareLaunchArgument(
                "cart_name",
                default_value="",
                description="Cart used to select cart-specific configuration.",
            ),
            DeclareLaunchArgument(
                "cart_config_path",
                default_value="",
                description="Optional explicit path to cart-specific YAML config.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz preset to open when use_rviz is true.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Open exactly one RViz window.",
            ),
            DeclareLaunchArgument(
                "start_velodyne",
                default_value="true",
                description="Start the Velodyne driver/transform stack.",
            ),
            DeclareLaunchArgument(
                "navigation_start_delay",
                default_value="5.0",
                description="Seconds to wait after /pcl_pose before navigation.",
            ),
            DeclareLaunchArgument(
                "enable_motor",
                default_value="true",
                description="Start motor_control unless set to false.",
            ),
            DeclareLaunchArgument(
                "enable_mola_auto_localization",
                default_value="false",
                description=(
                    "Start the conservative MOLA auto-localization "
                    "supervisor."
                ),
            ),
            DeclareLaunchArgument(
                "use_gps_relocalize",
                default_value="false",
                description=(
                    "Allow the MOLA supervisor to seed recovery relocalization "
                    "from GPS."
                ),
            ),
            DeclareLaunchArgument(
                "enable_usb_gps",
                default_value="false",
                description=(
                    "Start Garmin USB GPS driver and valid-fix filter."
                ),
            ),
            DeclareLaunchArgument(
                "gps_port",
                default_value="/dev/ttyACM0",
                description="Serial port for Garmin USB GPS.",
            ),
            DeclareLaunchArgument(
                "gps_baudrate",
                default_value="4800",
                description="Serial baud rate for Garmin USB GPS.",
            ),
            DeclareLaunchArgument(
                "gps_raw_topic",
                default_value="/fix",
                description="Raw Garmin GPS NavSatFix topic.",
            ),
            DeclareLaunchArgument(
                "gps_valid_topic",
                default_value="/fix_valid",
                description="Filtered valid NavSatFix topic.",
            ),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="",
                description=(
                    "Supervisor NavSatFix topic. Empty selects /gps normally "
                    "or gps_valid_topic when enable_usb_gps is true."
                ),
            ),
            DeclareLaunchArgument(
                "manual_pose_cooldown_sec",
                default_value="30.0",
                description="Suppress auto-recovery after RViz /initialpose.",
            ),
            DeclareLaunchArgument(
                "manual_pose_lockout_sec",
                default_value="30.0",
                description=(
                    "Hold auto-recovery after a manual pose is accepted by "
                    "MOLA health."
                ),
            ),
            DeclareLaunchArgument(
                "gps_fix_timeout_sec",
                default_value="3.0",
                description="Reject GPS messages with stale header stamps.",
            ),
            DeclareLaunchArgument(
                "gps_seed_max_age_sec",
                default_value="3.0",
                description="Maximum age for a cached GPS map seed.",
            ),
            DeclareLaunchArgument(
                "gps_max_covariance",
                default_value="100.0",
                description=(
                    "Maximum accepted NavSatFix covariance diagonal in m^2; "
                    "set <= 0 to disable."
                ),
            ),
            DeclareLaunchArgument(
                "gps_relocalize_cooldown_sec",
                default_value="8.0",
                description="Cooldown between GPS-seeded recovery requests.",
            ),
            DeclareLaunchArgument(
                "gps_yaw_sweep_enabled",
                default_value="false",
                description=(
                    "Try multiple GPS-seeded headings during supervisor "
                    "recovery."
                ),
            ),
            DeclareLaunchArgument(
                "gps_yaw_sweep_mode",
                default_value="absolute",
                description=(
                    "Yaw sweep candidate mode: absolute map-frame yaws or "
                    "relative offsets from the frozen seed yaw."
                ),
            ),
            DeclareLaunchArgument(
                "gps_yaw_sweep_accept_icp_quality",
                default_value="0.0",
                description=(
                    "Minimum ICP quality for GPS yaw-sweep diagnostic health; "
                    "0.0 keeps zero-valued outdoor diagnostics fallback-safe."
                ),
            ),
            DeclareLaunchArgument(
                "yaw_candidate_settle_sec",
                default_value="1.0",
                description="Seconds to let each yaw candidate settle.",
            ),
            DeclareLaunchArgument(
                "yaw_candidate_timeout_sec",
                default_value="7.0",
                description="Maximum seconds to wait for candidate evidence.",
            ),
            DeclareLaunchArgument(
                "post_recovery_lockout_sec",
                default_value="25.0",
                description="Suppress auto-recovery after accepted recovery.",
            ),
            DeclareLaunchArgument(
                "acceptance_near_seed_m",
                default_value="6.0",
                description=(
                    "Accept recovered MOLA pose only within this distance of "
                    "the frozen GPS seed."
                ),
            ),
            DeclareLaunchArgument(
                "acceptance_min_fresh_pose_count",
                default_value="3",
                description=(
                    "Fresh post-candidate MOLA pose samples required before "
                    "accepting GPS-assisted recovery."
                ),
            ),
            DeclareLaunchArgument(
                "acceptance_require_diagnostics_ok",
                default_value="true",
                description=(
                    "Require fresh healthy MOLA diagnostics before accepting "
                    "GPS-assisted recovery."
                ),
            ),
            DeclareLaunchArgument(
                "acceptance_require_match_quality",
                default_value="false",
                description=(
                    "Require direct MOLA match-quality evidence before "
                    "confirmed recovery acceptance."
                ),
            ),
            DeclareLaunchArgument(
                "acceptance_allow_quality_fallback",
                default_value="false",
                description=(
                    "Allow conservative stability fallback when diagnostics "
                    "do not expose a direct match-quality metric."
                ),
            ),
            DeclareLaunchArgument(
                "acceptance_confirmation_window_sec",
                default_value="2.0",
                description=(
                    "Continuous good-evidence duration required before "
                    "entering recovery lockout."
                ),
            ),
            DeclareLaunchArgument(
                "acceptance_require_pose_stability",
                default_value="true",
                description="Require pose stability during recovery confirmation.",
            ),
            DeclareLaunchArgument(
                "acceptance_pose_stability_window_sec",
                default_value="2.0",
                description="Seconds of pose history used for stability checks.",
            ),
            DeclareLaunchArgument(
                "acceptance_max_pose_drift_m",
                default_value="1.0",
                description="Maximum pose drift allowed in the stability window.",
            ),
            DeclareLaunchArgument(
                "acceptance_max_yaw_drift_rad",
                default_value="0.35",
                description="Maximum yaw drift allowed in the stability window.",
            ),
            DeclareLaunchArgument(
                "acceptance_min_icp_quality",
                default_value="0.75",
                description="Minimum accepted ICP/match quality when reported.",
            ),
            DeclareLaunchArgument(
                "acceptance_min_inlier_ratio",
                default_value="0.60",
                description="Minimum accepted inlier ratio when reported.",
            ),
            DeclareLaunchArgument(
                "acceptance_min_matched_points",
                default_value="0",
                description="Minimum matched/inlier point count; 0 disables.",
            ),
            DeclareLaunchArgument(
                "acceptance_max_icp_error",
                default_value="0.75",
                description="Maximum accepted ICP/alignment error when reported.",
            ),
            DeclareLaunchArgument(
                "acceptance_max_pose_covariance",
                default_value="4.0",
                description="Maximum accepted pose covariance when reported.",
            ),
            DeclareLaunchArgument(
                "acceptance_max_dropped_frames",
                default_value="0.10",
                description="Maximum accepted dropped-frame metric when reported.",
            ),
            DeclareLaunchArgument(
                "acceptance_required_diagnostic_level",
                default_value="0",
                description="Maximum accepted diagnostic level; 0 means OK only.",
            ),
            DeclareLaunchArgument(
                "recovery_require_sustained_loss",
                default_value="true",
                description=(
                    "Require localization loss to persist before recovery "
                    "relocalization can start."
                ),
            ),
            DeclareLaunchArgument(
                "recovery_min_lost_duration_sec",
                default_value="3.0",
                description="Minimum sustained lost duration before recovery.",
            ),
            DeclareLaunchArgument(
                "motor_port",
                default_value="/dev/ttyUSB0",
                description="Serial device used by motor_endpoint.",
            ),
            DeclareLaunchArgument(
                "motor_baudrate",
                default_value="57600",
                description="Serial baud rate used by motor_endpoint.",
            ),
            DeclareLaunchArgument(
                "enable_aad",
                default_value="false",
                description="Enable anomaly logging nodes.",
            ),
            DeclareLaunchArgument(
                "graph_dir",
                default_value=navigation_maps_dir,
                description="Directory containing navigation graph files.",
            ),
            DeclareLaunchArgument(
                "graph_file",
                default_value="main_shift3_gps.gml",
                description="Navigation graph file.",
            ),
            DeclareLaunchArgument(
                "graph_coordinate_format",
                default_value="gps",
                description='Graph coordinate format: "gps" or "ros".',
            ),
            DeclareLaunchArgument(
                "calibration_config_dir",
                default_value=navigation_maps_dir,
                description="Directory containing landmark calibration YAML.",
            ),
            DeclareLaunchArgument(
                "calibration_config_file",
                default_value="with_gps2_adjusted_route.yaml",
                description="Landmark calibration YAML file.",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value="/root/dev_ws/maps/with_gps2.mm",
                description="Georeferenced MOLA map file.",
            ),
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/velodyne_points",
                description="Live LiDAR PointCloud2 topic.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Robot base frame used by MOLA.",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="velodyne",
                description="LiDAR frame for the static sensor transform.",
            ),
            DeclareLaunchArgument(
                "lidar_x",
                default_value="0.5",
                description="Shared/default LiDAR x offset when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_y",
                default_value="0.0",
                description="Shared/default LiDAR y offset when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_z",
                default_value="1.75",
                description="Shared/default LiDAR z offset when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_yaw",
                default_value="0.0",
                description="Shared/default LiDAR yaw when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_pitch",
                default_value="0.0",
                description="Shared/default LiDAR pitch when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_roll",
                default_value="0.0",
                description="Shared/default LiDAR roll when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "publish_odom_tf",
                default_value="true",
                description="Publish dynamic TF from MOLA odometry.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cart_launch_path),
                launch_arguments=forwarded_args.items(),
            ),
        ]
    )
