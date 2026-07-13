import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _source_or_share(source_path, package_name, *share_parts):
    if os.path.exists(source_path):
        return source_path
    return os.path.join(get_package_share_directory(package_name), *share_parts)


def generate_launch_description():
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

    launch_rviz = LaunchConfiguration("launch_rviz")
    enable_motor = LaunchConfiguration("enable_motor")
    enable_aad = LaunchConfiguration("enable_aad")
    enable_mola_auto_localization = LaunchConfiguration(
        "enable_mola_auto_localization"
    )
    enable_usb_gps = LaunchConfiguration("enable_usb_gps")
    gps_port = LaunchConfiguration("gps_port")
    gps_baudrate = LaunchConfiguration("gps_baudrate")
    gps_raw_topic = LaunchConfiguration("gps_raw_topic")
    gps_valid_topic = LaunchConfiguration("gps_valid_topic")
    effective_gps_topic = PythonExpression(
        [
            "'",
            LaunchConfiguration("gps_topic"),
            "' or ('",
            gps_valid_topic,
            "' if '",
            enable_usb_gps,
            "'.lower() in ('1', 'true', 'yes', 'on') else '/gps')",
        ]
    )
    use_gps_relocalize = LaunchConfiguration("use_gps_relocalize")
    gps_yaw_sweep_enabled = LaunchConfiguration("gps_yaw_sweep_enabled")
    gps_fix_timeout_sec = LaunchConfiguration("gps_fix_timeout_sec")
    gps_seed_max_age_sec = LaunchConfiguration("gps_seed_max_age_sec")
    gps_max_covariance = LaunchConfiguration("gps_max_covariance")
    gps_relocalize_cooldown_sec = LaunchConfiguration(
        "gps_relocalize_cooldown_sec"
    )
    manual_pose_cooldown_sec = LaunchConfiguration("manual_pose_cooldown_sec")
    manual_pose_lockout_sec = LaunchConfiguration("manual_pose_lockout_sec")
    gps_yaw_sweep_mode = LaunchConfiguration("gps_yaw_sweep_mode")
    gps_yaw_sweep_accept_icp_quality = LaunchConfiguration(
        "gps_yaw_sweep_accept_icp_quality"
    )
    yaw_candidate_settle_sec = LaunchConfiguration("yaw_candidate_settle_sec")
    yaw_candidate_timeout_sec = LaunchConfiguration("yaw_candidate_timeout_sec")
    post_recovery_lockout_sec = LaunchConfiguration(
        "post_recovery_lockout_sec"
    )
    acceptance_near_seed_m = LaunchConfiguration("acceptance_near_seed_m")
    acceptance_min_fresh_pose_count = LaunchConfiguration(
        "acceptance_min_fresh_pose_count"
    )
    acceptance_require_diagnostics_ok = LaunchConfiguration(
        "acceptance_require_diagnostics_ok"
    )
    acceptance_require_match_quality = LaunchConfiguration(
        "acceptance_require_match_quality"
    )
    acceptance_allow_quality_fallback = LaunchConfiguration(
        "acceptance_allow_quality_fallback"
    )
    acceptance_confirmation_window_sec = LaunchConfiguration(
        "acceptance_confirmation_window_sec"
    )
    acceptance_require_pose_stability = LaunchConfiguration(
        "acceptance_require_pose_stability"
    )
    acceptance_pose_stability_window_sec = LaunchConfiguration(
        "acceptance_pose_stability_window_sec"
    )
    acceptance_max_pose_drift_m = LaunchConfiguration(
        "acceptance_max_pose_drift_m"
    )
    acceptance_max_yaw_drift_rad = LaunchConfiguration(
        "acceptance_max_yaw_drift_rad"
    )
    acceptance_min_icp_quality = LaunchConfiguration(
        "acceptance_min_icp_quality"
    )
    acceptance_min_inlier_ratio = LaunchConfiguration(
        "acceptance_min_inlier_ratio"
    )
    acceptance_min_matched_points = LaunchConfiguration(
        "acceptance_min_matched_points"
    )
    acceptance_max_icp_error = LaunchConfiguration(
        "acceptance_max_icp_error"
    )
    acceptance_max_pose_covariance = LaunchConfiguration(
        "acceptance_max_pose_covariance"
    )
    acceptance_max_dropped_frames = LaunchConfiguration(
        "acceptance_max_dropped_frames"
    )
    acceptance_required_diagnostic_level = LaunchConfiguration(
        "acceptance_required_diagnostic_level"
    )
    recovery_require_sustained_loss = LaunchConfiguration(
        "recovery_require_sustained_loss"
    )
    recovery_min_lost_duration_sec = LaunchConfiguration(
        "recovery_min_lost_duration_sec"
    )

    mola_auto_localization_params = PathJoinSubstitution(
        [
            FindPackageShare("localization_launch"),
            "param",
            "mola_auto_localization_supervisor.yaml",
        ]
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("localization_launch"),
                "/launch/velodyne_only.launch.py",
            ]
        ),
        launch_arguments={
            "cart": LaunchConfiguration("cart"),
            "cart_name": LaunchConfiguration("cart_name"),
            "cart_config_path": LaunchConfiguration("cart_config_path"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_velodyne")),
    )

    mola_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("localization_launch"),
                "/launch/mola_map_localization.launch.py",
            ]
        ),
        launch_arguments={
            "use_rviz": "false",
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
            "cart": LaunchConfiguration("cart"),
            "cart_name": LaunchConfiguration("cart_name"),
            "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
            "cart_config_path": LaunchConfiguration("cart_config_path"),
        }.items(),
    )

    pcl_pose_relay = Node(
        package="localization_launch",
        executable="pcl_pose_relay",
        name="pcl_pose_relay",
        output="screen",
    )

    usb_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("localization_launch"),
                "/launch/garmin_gps18x_valid.launch.py",
            ]
        ),
        launch_arguments={
            "port": gps_port,
            "baud": gps_baudrate,
            "input_topic": gps_raw_topic,
            "output_topic": gps_valid_topic,
        }.items(),
        condition=IfCondition(enable_usb_gps),
    )

    usb_gps_config_log = LogInfo(
        msg=[
            "USB_GPS_CONFIG enabled=",
            enable_usb_gps,
            " port=",
            gps_port,
            " baudrate=",
            gps_baudrate,
            " raw_topic=",
            gps_raw_topic,
            " valid_topic=",
            gps_valid_topic,
        ]
    )

    gps_topic_config_log = LogInfo(
        msg=[
            "GPS_TOPIC_CONFIG gps_topic=",
            effective_gps_topic,
            " use_gps_relocalize=",
            use_gps_relocalize,
            " gps_yaw_sweep_enabled=",
            gps_yaw_sweep_enabled,
        ]
    )

    mola_auto_localization_supervisor = Node(
        package="localization_launch",
        executable="mola_auto_localization_supervisor",
        name="mola_auto_localization_supervisor",
        output="screen",
        parameters=[
            mola_auto_localization_params,
            {
                "cloud_topic": LaunchConfiguration("lidar_topic"),
                "mola_pose_topic": "/lidar_odometry/pose",
                "use_gps_relocalize": use_gps_relocalize,
                "gps_topic": effective_gps_topic,
                "gps_yaw_sweep_enabled": gps_yaw_sweep_enabled,
                "manual_pose_cooldown_sec": ParameterValue(
                    manual_pose_cooldown_sec,
                    value_type=float,
                ),
                "manual_pose_lockout_sec": ParameterValue(
                    manual_pose_lockout_sec,
                    value_type=float,
                ),
                "gps_fix_timeout_sec": ParameterValue(
                    gps_fix_timeout_sec,
                    value_type=float,
                ),
                "gps_seed_max_age_sec": ParameterValue(
                    gps_seed_max_age_sec,
                    value_type=float,
                ),
                "gps_max_covariance": ParameterValue(
                    gps_max_covariance,
                    value_type=float,
                ),
                "gps_relocalize_cooldown_sec": ParameterValue(
                    gps_relocalize_cooldown_sec,
                    value_type=float,
                ),
                "gps_yaw_sweep_mode": gps_yaw_sweep_mode,
                "gps_yaw_sweep_accept_icp_quality": ParameterValue(
                    gps_yaw_sweep_accept_icp_quality,
                    value_type=float,
                ),
                "yaw_candidate_settle_sec": ParameterValue(
                    yaw_candidate_settle_sec,
                    value_type=float,
                ),
                "yaw_candidate_timeout_sec": ParameterValue(
                    yaw_candidate_timeout_sec,
                    value_type=float,
                ),
                "post_recovery_lockout_sec": ParameterValue(
                    post_recovery_lockout_sec,
                    value_type=float,
                ),
                "acceptance_near_seed_m": ParameterValue(
                    acceptance_near_seed_m,
                    value_type=float,
                ),
                "acceptance_min_fresh_pose_count": ParameterValue(
                    acceptance_min_fresh_pose_count,
                    value_type=int,
                ),
                "acceptance_require_diagnostics_ok": (
                    acceptance_require_diagnostics_ok
                ),
                "acceptance_require_match_quality": (
                    acceptance_require_match_quality
                ),
                "acceptance_allow_quality_fallback": (
                    acceptance_allow_quality_fallback
                ),
                "acceptance_confirmation_window_sec": ParameterValue(
                    acceptance_confirmation_window_sec,
                    value_type=float,
                ),
                "acceptance_require_pose_stability": (
                    acceptance_require_pose_stability
                ),
                "acceptance_pose_stability_window_sec": ParameterValue(
                    acceptance_pose_stability_window_sec,
                    value_type=float,
                ),
                "acceptance_max_pose_drift_m": ParameterValue(
                    acceptance_max_pose_drift_m,
                    value_type=float,
                ),
                "acceptance_max_yaw_drift_rad": ParameterValue(
                    acceptance_max_yaw_drift_rad,
                    value_type=float,
                ),
                "acceptance_min_icp_quality": ParameterValue(
                    acceptance_min_icp_quality,
                    value_type=float,
                ),
                "acceptance_min_inlier_ratio": ParameterValue(
                    acceptance_min_inlier_ratio,
                    value_type=float,
                ),
                "acceptance_min_matched_points": ParameterValue(
                    acceptance_min_matched_points,
                    value_type=int,
                ),
                "acceptance_max_icp_error": ParameterValue(
                    acceptance_max_icp_error,
                    value_type=float,
                ),
                "acceptance_max_pose_covariance": ParameterValue(
                    acceptance_max_pose_covariance,
                    value_type=float,
                ),
                "acceptance_max_dropped_frames": ParameterValue(
                    acceptance_max_dropped_frames,
                    value_type=float,
                ),
                "acceptance_required_diagnostic_level": ParameterValue(
                    acceptance_required_diagnostic_level,
                    value_type=int,
                ),
                "recovery_require_sustained_loss": (
                    recovery_require_sustained_loss
                ),
                "recovery_min_lost_duration_sec": ParameterValue(
                    recovery_min_lost_duration_sec,
                    value_type=float,
                ),
                "calibration_config_dir": LaunchConfiguration(
                    "calibration_config_dir"
                ),
                "calibration_config_file": LaunchConfiguration(
                    "calibration_config_file"
                ),
            },
        ],
        condition=IfCondition(enable_mola_auto_localization),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("navigation"), "/launch/navigation.launch.py"]
        ),
        launch_arguments={
            "graph_dir": LaunchConfiguration("graph_dir"),
            "graph_file": LaunchConfiguration("graph_file"),
            "graph_coordinate_format": LaunchConfiguration(
                "graph_coordinate_format"
            ),
            "calibration_config_dir": LaunchConfiguration(
                "calibration_config_dir"
            ),
            "calibration_config_file": LaunchConfiguration(
                "calibration_config_file"
            ),
            "enable_aad": enable_aad,
        }.items(),
    )

    rviz_node = ExecuteProcess(
        cmd=["rviz2", "-d", LaunchConfiguration("rviz_config")],
        name="rviz2_mola_localization",
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    rosbridge_cleanup = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "pkill -f '[r]osbridge_websocket' || true; "
                "pkill -f '[r]osapi_node' || true; "
                "pkill -f '[r]osbridge_server rosbridge_websocket_launch.xml' || true; "
                "sleep 1"
            ),
        ],
        name="cleanup_stale_rosbridge",
        output="screen",
    )

    rosbridge_launch = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    (
                        "source /opt/ros/jazzy/setup.bash && "
                        "source /root/dev_ws/install/setup.bash && "
                        "exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
                    ),
                ],
                name="rosbridge_manual_launch",
                output="screen",
            )
        ],
    )

    motor_launch = Node(
        package="motor_control",
        executable="motor_endpoint",
        name="motor_endpoint",
        output="screen",
        parameters=[
            {
                "arduino_port": LaunchConfiguration("motor_port"),
                "baudrate": ParameterValue(
                    LaunchConfiguration("motor_baudrate"),
                    value_type=int,
                ),
            }
        ],
        condition=IfCondition(enable_motor),
    )

    wait_for_pcl_pose = ExecuteProcess(
        cmd=["ros2", "topic", "echo", "/pcl_pose", "--once"],
        name="wait_for_pcl_pose",
        output="screen",
    )

    delayed_route_stack = TimerAction(
        period=LaunchConfiguration("navigation_start_delay"),
        actions=[
            LogInfo(msg="Starting navigation route stack."),
            navigation_launch,
            motor_launch,
        ],
    )

    start_navigation_after_pcl_pose = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_pcl_pose,
            on_exit=[
                LogInfo(
                    msg=(
                        "Observed /pcl_pose; waiting for RViz subscribers "
                        "before starting navigation."
                    )
                ),
                delayed_route_stack,
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart",
                default_value="madison",
                description=(
                    "Compatibility cart selector: james or madison. "
                    "Prefer cart_name for new commands."
                ),
            ),
            DeclareLaunchArgument(
                "cart_name",
                default_value="",
                description=(
                    "Cart used to select cart-specific localization and "
                    "sensor configuration: james or madison."
                ),
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
                "launch_rviz",
                default_value="true",
                description="Open the JACart MOLA RViz preset window.",
            ),
            DeclareLaunchArgument(
                "start_velodyne",
                default_value="true",
                description=(
                    "Start the Velodyne driver/transform stack that publishes "
                    "/velodyne_points."
                ),
            ),
            DeclareLaunchArgument(
                "navigation_start_delay",
                default_value="5.0",
                description=(
                    "Seconds to wait after /pcl_pose before starting "
                    "navigation, giving RViz time to subscribe."
                ),
            ),
            DeclareLaunchArgument(
                "enable_motor",
                default_value="true",
                description=(
                    "Start motor_control by default; set enable_motor:=false "
                    "for safe no-motor testing."
                ),
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
            usb_gps_config_log,
            gps_topic_config_log,
            usb_gps_launch,
            velodyne_launch,
            mola_localization_launch,
            pcl_pose_relay,
            mola_auto_localization_supervisor,
            rviz_node,
            rosbridge_cleanup,
            rosbridge_launch,
            wait_for_pcl_pose,
            start_navigation_after_pcl_pose,
        ]
    )
