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
    declare_radar_parent_frame = DeclareLaunchArgument(
        "radar_parent_frame",
        default_value="base_link",
        description="Parent TF frame for the TI radar static transform.",
    )
    declare_radar_frame = DeclareLaunchArgument(
        "radar_frame",
        default_value="ti_mmwave_0",
        description="Child TF frame for the TI radar static transform.",
    )
    declare_radar_x = DeclareLaunchArgument(
        "radar_x",
        default_value="0.0",
        description="Radar X offset in meters relative to radar_parent_frame.",
    )
    declare_radar_y = DeclareLaunchArgument(
        "radar_y",
        default_value="0.0",
        description="Radar Y offset in meters relative to radar_parent_frame.",
    )
    declare_radar_z = DeclareLaunchArgument(
        "radar_z",
        default_value="0.0",
        description="Radar Z offset in meters relative to radar_parent_frame.",
    )
    declare_radar_roll = DeclareLaunchArgument(
        "radar_roll",
        default_value="0.0",
        description="Radar roll in radians relative to radar_parent_frame.",
    )
    declare_radar_pitch = DeclareLaunchArgument(
        "radar_pitch",
        default_value="0.0",
        description="Radar pitch in radians relative to radar_parent_frame.",
    )
    declare_radar_yaw = DeclareLaunchArgument(
        "radar_yaw",
        default_value="0.0",
        description="Radar yaw in radians relative to radar_parent_frame.",
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
        )
    )

    # Launch the motor control launcher
    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("motor_control"), "/launch/motor.launch.py"]
        )
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

    #start rosbridge
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket_server',
        output='screen',
        parameters=[],
    )

    radar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="radar_tf",
        arguments=[
            LaunchConfiguration("radar_x"),
            LaunchConfiguration("radar_y"),
            LaunchConfiguration("radar_z"),
            LaunchConfiguration("radar_roll"),
            LaunchConfiguration("radar_pitch"),
            LaunchConfiguration("radar_yaw"),
            LaunchConfiguration("radar_parent_frame"),
            LaunchConfiguration("radar_frame"),
        ],
        condition=IfCondition(enable_radar),
        output="screen",
    )

    radar_launch = OpaqueFunction(function=_include_radar_launch)

    delayed_stack = TimerAction(
        period=console_start_delay_s,
        actions=[
            localization_launch,
            navigation_launch,
            motor_control_launch,
            radar_tf,
            radar_launch,
            rviz2_command,
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
            declare_radar_parent_frame,
            declare_radar_frame,
            declare_radar_x,
            declare_radar_y,
            declare_radar_z,
            declare_radar_roll,
            declare_radar_pitch,
            declare_radar_yaw,
            swri_console_node,
            delayed_stack,
        ]
    )
