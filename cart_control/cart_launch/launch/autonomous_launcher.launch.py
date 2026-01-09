from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory 
import os
import launch_ros
import launch_ros.actions
import launch_ros.events


def generate_launch_description():

    console_start_delay_s = LaunchConfiguration("console_start_delay_s")

    declare_console_start_delay_s = DeclareLaunchArgument(
        "console_start_delay_s",
        default_value="5.0",
        description="Delay (seconds) before launching the rest of the stack, to let swri_console start first.",
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
        )
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

    delayed_stack = TimerAction(
        period=console_start_delay_s,
        actions=[
            localization_launch,
            navigation_launch,
            motor_control_launch,
            rviz2_command,
            rosbridge_node,
        ],
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            declare_console_start_delay_s,
            swri_console_node,
            delayed_stack,
        ]
    )
