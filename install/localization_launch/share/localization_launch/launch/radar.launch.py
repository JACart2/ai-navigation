from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    frame_id = LaunchConfiguration("radar_frame_id")
    default_cfg = os.path.join(
        get_package_share_directory("localization_launch"),
        "config",
        "profile_2d.cfg",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "radar_cli_port",
                default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if00-port0",
            ),
            DeclareLaunchArgument(
                "radar_data_port",
                default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if01-port0",
            ),
            DeclareLaunchArgument(
                "radar_cfg_file",
                default_value=default_cfg,
            ),
            DeclareLaunchArgument("radar_frame_id", default_value="radar_link"),
            DeclareLaunchArgument("radar_x", default_value="0.0"),
            DeclareLaunchArgument("radar_y", default_value="0.0"),
            DeclareLaunchArgument("radar_z", default_value="0.0"),
            DeclareLaunchArgument("radar_roll", default_value="0.0"),
            DeclareLaunchArgument("radar_pitch", default_value="0.0"),
            DeclareLaunchArgument("radar_yaw", default_value="0.0"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="radar_tf",
                arguments=[
                    "--x",
                    LaunchConfiguration("radar_x"),
                    "--y",
                    LaunchConfiguration("radar_y"),
                    "--z",
                    LaunchConfiguration("radar_z"),
                    "--roll",
                    LaunchConfiguration("radar_roll"),
                    "--pitch",
                    LaunchConfiguration("radar_pitch"),
                    "--yaw",
                    LaunchConfiguration("radar_yaw"),
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    frame_id,
                ],
                output="screen",
            ),
            Node(
                package="localization_launch",
                executable="iwr6843_radar_node",
                name="iwr6843_radar_node",
                output="screen",
                parameters=[
                    {
                        "cli_port": LaunchConfiguration("radar_cli_port"),
                        "data_port": LaunchConfiguration("radar_data_port"),
                        "cfg_file": LaunchConfiguration("radar_cfg_file"),
                        "frame_id": frame_id,
                    }
                ],
            ),
        ]
    )
