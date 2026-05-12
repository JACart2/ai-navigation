import os
import shlex

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include_radar_launch(context, *args, **kwargs):
    enable_radar = (
        perform_substitutions(context, [LaunchConfiguration("enable_radar")])
        .strip()
        .lower()
    )
    if enable_radar not in ("1", "true", "yes", "on"):
        return []

    radar_cfg_file = perform_substitutions(
        context, [LaunchConfiguration("radar_cfg_file")]
    ).strip()
    radar_command_port = perform_substitutions(
        context, [LaunchConfiguration("radar_command_port")]
    ).strip()
    radar_data_port = perform_substitutions(
        context, [LaunchConfiguration("radar_data_port")]
    ).strip()
    radar_setup_bash = perform_substitutions(
        context, [LaunchConfiguration("radar_setup_bash")]
    ).strip()

    try:
        radar_share = get_package_share_directory("ti_mmwave_rospkg")
    except PackageNotFoundError:
        radar_share = None

    if radar_share:
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(radar_share, "launch", "6843AOP_Standard.py")
                ),
                launch_arguments={
                    "rviz": "false",
                    "cfg_file": radar_cfg_file,
                    "command_port": radar_command_port,
                    "data_port": radar_data_port,
                }.items(),
            )
        ]

    setup_candidates = []
    if radar_setup_bash:
        setup_candidates.append(radar_setup_bash)
    setup_candidates.extend(
        [
            "/root/dev_ws/src/mmwave_ti_ros/ros2_driver/install/setup.bash",
            "/home/collision-avoidance/dev_ws/src/mmwave_ti_ros/ros2_driver/install/setup.bash",
        ]
    )

    for setup_path in setup_candidates:
        if setup_path and os.path.exists(setup_path):
            radar_launch_cmd = (
                f"source {shlex.quote(setup_path)} && "
                "ros2 launch ti_mmwave_rospkg 6843AOP_Standard.py "
                f"rviz:=false cfg_file:={shlex.quote(radar_cfg_file)} "
                f"command_port:={shlex.quote(radar_command_port)} "
                f"data_port:={shlex.quote(radar_data_port)}"
            )
            return [
                ExecuteProcess(
                    cmd=["bash", "-lc", radar_launch_cmd],
                    output="screen",
                )
            ]

    return [
        LogInfo(
            msg=(
                "Radar launch skipped because ti_mmwave_rospkg is not in the current "
                "environment and no radar setup.bash overlay was found."
            )
        )
    ]


def generate_launch_description():

    console_start_delay_s = LaunchConfiguration("console_start_delay_s")
    cart_config_path = LaunchConfiguration("cart_config_path")
    enable_radar = LaunchConfiguration("enable_radar")
    motor_baudrate = LaunchConfiguration("motor_baudrate")
    motor_arduino_port = LaunchConfiguration("motor_arduino_port")
    radar_min_height = LaunchConfiguration("radar_min_height")
    radar_max_height = LaunchConfiguration("radar_max_height")
    radar_min_cluster_size = LaunchConfiguration("radar_min_cluster_size")
    radar_persistence_frames = LaunchConfiguration("radar_persistence_frames")
    radar_persistence_match_radius = LaunchConfiguration("radar_persistence_match_radius")
    radar_track_timeout_sec = LaunchConfiguration("radar_track_timeout_sec")
    radar_filter_x_min = LaunchConfiguration("radar_filter_x_min")
    radar_filter_x_max = LaunchConfiguration("radar_filter_x_max")
    radar_filter_y_min = LaunchConfiguration("radar_filter_y_min")
    radar_filter_y_max = LaunchConfiguration("radar_filter_y_max")
    radar_filter_z_min = LaunchConfiguration("radar_filter_z_min")
    radar_filter_z_max = LaunchConfiguration("radar_filter_z_max")
    radar_filter_gui = LaunchConfiguration("radar_filter_gui")

    declare_console_start_delay_s = DeclareLaunchArgument(
        "console_start_delay_s",
        default_value="5.0",
        description="Delay (seconds) before launching the rest of the stack, to let swri_console start first.",
    )
    declare_cart_config_path = DeclareLaunchArgument(
        "cart_config_path",
        default_value=os.path.join(
            get_package_share_directory("cart_launch"), "config", "cart_james.yaml"
        ),
        description="Path to cart-specific YAML config (must contain zed_front_serial and zed_rear_serial)",
    )
    declare_enable_radar = DeclareLaunchArgument(
        "enable_radar",
        default_value="true",
        description="Launch the TI IWR6843AOP radar alongside the autonomous stack.",
    )
    declare_radar_cfg_file = DeclareLaunchArgument(
        "radar_cfg_file",
        default_value="6843AOP_Standard.cfg",
        description="TI mmWave radar configuration file to pass into 6843AOP_Standard.py.",
    )
    declare_radar_command_port = DeclareLaunchArgument(
        "radar_command_port",
        default_value="/dev/ttyUSB0",
        description="Serial port used by the TI radar command interface.",
    )
    declare_radar_data_port = DeclareLaunchArgument(
        "radar_data_port",
        default_value="/dev/ttyUSB1",
        description="Serial port used by the TI radar data interface.",
    )
    declare_radar_setup_bash = DeclareLaunchArgument(
        "radar_setup_bash",
        default_value="",
        description=(
            "Optional setup.bash for a TI radar overlay if ti_mmwave_rospkg is not already "
            "in the sourced ROS environment."
        ),
    )
    declare_motor_baudrate = DeclareLaunchArgument(
        "motor_baudrate",
        default_value="57600",
        description="Baudrate used by the motor controller Arduino serial interface.",
    )
    declare_motor_arduino_port = DeclareLaunchArgument(
        "motor_arduino_port",
        default_value="/dev/ttyUSB0",
        description="Serial port used by the motor controller Arduino interface.",
    )
    declare_radar_min_height = DeclareLaunchArgument(
        "radar_min_height",
        default_value="-0.10",
        description="Minimum radar point height kept during PointCloud2->LaserScan conversion.",
    )
    declare_radar_max_height = DeclareLaunchArgument(
        "radar_max_height",
        default_value="0.75",
        description="Maximum radar point height kept during PointCloud2->LaserScan conversion.",
    )
    declare_radar_min_cluster_size = DeclareLaunchArgument(
        "radar_min_cluster_size",
        default_value="2",
        description="Minimum number of adjacent radar points required to form an obstacle cluster.",
    )
    declare_radar_persistence_frames = DeclareLaunchArgument(
        "radar_persistence_frames",
        default_value="3",
        description="Number of consecutive radar frames an object must persist before publishing.",
    )
    declare_radar_persistence_match_radius = DeclareLaunchArgument(
        "radar_persistence_match_radius",
        default_value="0.75",
        description="Maximum XY distance in meters for associating radar detections across frames.",
    )
    declare_radar_track_timeout_sec = DeclareLaunchArgument(
        "radar_track_timeout_sec",
        default_value="0.40",
        description="How long to retain an unmatched radar track while waiting for persistence.",
    )
    declare_radar_filter_x_min = DeclareLaunchArgument(
        "radar_filter_x_min", default_value="0.25",
        description="Radar XYZ filter: minimum x in radar frame (meters).",
    )
    declare_radar_filter_x_max = DeclareLaunchArgument(
        "radar_filter_x_max", default_value="3.58",
        description="Radar XYZ filter: maximum x in radar frame (meters).",
    )
    declare_radar_filter_y_min = DeclareLaunchArgument(
        "radar_filter_y_min", default_value="-2.10",
        description="Radar XYZ filter: minimum y in radar frame (meters).",
    )
    declare_radar_filter_y_max = DeclareLaunchArgument(
        "radar_filter_y_max", default_value="5.00",
        description="Radar XYZ filter: maximum y in radar frame (meters).",
    )
    declare_radar_filter_z_min = DeclareLaunchArgument(
        "radar_filter_z_min", default_value="-0.65",
        description="Radar XYZ filter: minimum z in radar frame (meters). Defaults filter out the ground.",
    )
    declare_radar_filter_z_max = DeclareLaunchArgument(
        "radar_filter_z_max", default_value="1.11",
        description="Radar XYZ filter: maximum z in radar frame (meters).",
    )
    declare_radar_filter_gui = DeclareLaunchArgument(
        "radar_filter_gui", default_value="true",
        description="Open the Tk slider window for adjusting the radar XYZ filter live.",
    )
    declare_enable_aad = DeclareLaunchArgument(
        "enable_aad",
        default_value="true",
        description="Enable anomaly logging",
    )

    swri_console_node = Node(
        package="swri_console",
        executable="swri_console",
        name="swri_console",
        output="screen",
    )

    # Launch the localization launcher
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("localization_launch"),
                "/launch/localization_full_launcher.launch.py",
            ]
        ),
        launch_arguments={
            "cart_config_path": cart_config_path,
        }.items(),
    )

    # Launch the navigation launcher
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("navigation"), "/launch/navigation.launch.py"]
        ),
        launch_arguments={
            "radar_min_height": radar_min_height,
            "radar_max_height": radar_max_height,
            "radar_min_cluster_size": radar_min_cluster_size,
            "radar_persistence_frames": radar_persistence_frames,
            "radar_persistence_match_radius": radar_persistence_match_radius,
            "radar_track_timeout_sec": radar_track_timeout_sec,
            "radar_filter_x_min": radar_filter_x_min,
            "radar_filter_x_max": radar_filter_x_max,
            "radar_filter_y_min": radar_filter_y_min,
            "radar_filter_y_max": radar_filter_y_max,
            "radar_filter_z_min": radar_filter_z_min,
            "radar_filter_z_max": radar_filter_z_max,
            "radar_filter_gui": radar_filter_gui,
            "enable_aad": LaunchConfiguration("enable_aad"),
        }.items(),
    )

    # Launch the motor control launcher
    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("motor_control"), "/launch/motor.launch.py"]
        ),
        launch_arguments={
            "baudrate": motor_baudrate,
            "arduino_port": motor_arduino_port,
            "enable_aad": LaunchConfiguration("enable_aad"),
        }.items(),
    )

    # Execute the RViz2 command with the specified configuration file
    rviz2_command = ExecuteProcess(
        cmd=[
            "rviz2",
            "-d",
            os.path.join(
                get_package_share_directory("cart_launch"), "rviz", "localization.rviz"
            ),
        ],
        shell=True,
    )

    # Second RViz window — locked to the radar's frame so we can see the radar POV
    # alongside the map view. Same data, different camera.
    rviz2_radar_pov = ExecuteProcess(
        cmd=[
            "rviz2",
            "-d",
            os.path.join(
                get_package_share_directory("cart_launch"), "rviz", "radar_pov.rviz"
            ),
        ],
        shell=True,
    )

    #start rosbridge
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket_server',
        output='screen',
        parameters=[],
    )

    radar_launch = OpaqueFunction(function=_include_radar_launch)

    delayed_stack = TimerAction(
        period=console_start_delay_s,
        actions=[
            localization_launch,
            navigation_launch,
            motor_control_launch,
            radar_launch,
            rviz2_command,
            rviz2_radar_pov,
            rosbridge_node,
        ],
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            declare_console_start_delay_s,
            declare_cart_config_path,
            declare_enable_radar,
            declare_radar_cfg_file,
            declare_radar_command_port,
            declare_radar_data_port,
            declare_radar_setup_bash,
            declare_motor_baudrate,
            declare_motor_arduino_port,
            declare_radar_min_height,
            declare_radar_max_height,
            declare_radar_min_cluster_size,
            declare_radar_persistence_frames,
            declare_radar_persistence_match_radius,
            declare_radar_track_timeout_sec,
            declare_radar_filter_x_min,
            declare_radar_filter_x_max,
            declare_radar_filter_y_min,
            declare_radar_filter_y_max,
            declare_radar_filter_z_min,
            declare_radar_filter_z_max,
            declare_radar_filter_gui,
            declare_enable_aad,
            swri_console_node,
            delayed_stack,
        ]
    )
