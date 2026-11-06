import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mola_autonomy_launch = os.path.join(
        get_package_share_directory("cart_launch"),
        "launch",
        "mola_autonomy.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_mola_auto_localization",
                default_value="false",
                description=(
                    "Start the conservative LiDAR-first MOLA auto-localization "
                    "supervisor."
                ),
            ),
            DeclareLaunchArgument(
                "enable_aad_camera_capture",
                default_value="true",
                description=(
                    "Optionally subscribe to raw cameras for bounded AAD "
                    "stop-event context."
                ),
            ),
            DeclareLaunchArgument(
                "start_cameras",
                default_value="true",
                description="Optionally start the front and rear ZED camera nodes.",
            ),
            DeclareLaunchArgument(
                "enable_gps_recovery",
                default_value="false",
                description="Use GPS as a recovery hint after localization is lost.",
            ),
            DeclareLaunchArgument("enable_gps_auto_anchor", default_value="true"),
            DeclareLaunchArgument("gps_fix_topic", default_value="/fix"),
            DeclareLaunchArgument("gps_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("gps_baud", default_value="4800"),
            DeclareLaunchArgument("gps_frame", default_value="gps"),
            DeclareLaunchArgument("gps_map_origin_lat", default_value="0.0"),
            DeclareLaunchArgument("gps_map_origin_lon", default_value="0.0"),
            DeclareLaunchArgument("gps_map_origin_alt", default_value="0.0"),
            DeclareLaunchArgument("gps_map_yaw_from_enu", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mola_autonomy_launch),
                launch_arguments={
                    "cart": "james",
                    "enable_mola_auto_localization": LaunchConfiguration(
                        "enable_mola_auto_localization"
                    ),
                    "enable_aad_camera_capture": LaunchConfiguration(
                        "enable_aad_camera_capture"
                    ),
                    "start_cameras": LaunchConfiguration("start_cameras"),
                    "enable_gps_recovery": LaunchConfiguration(
                        "enable_gps_recovery"
                    ),
                    "enable_gps_auto_anchor": LaunchConfiguration(
                        "enable_gps_auto_anchor"
                    ),
                    "gps_fix_topic": LaunchConfiguration("gps_fix_topic"),
                    "gps_port": LaunchConfiguration("gps_port"),
                    "gps_baud": LaunchConfiguration("gps_baud"),
                    "gps_frame": LaunchConfiguration("gps_frame"),
                    "gps_map_origin_lat": LaunchConfiguration(
                        "gps_map_origin_lat"
                    ),
                    "gps_map_origin_lon": LaunchConfiguration(
                        "gps_map_origin_lon"
                    ),
                    "gps_map_origin_alt": LaunchConfiguration(
                        "gps_map_origin_alt"
                    ),
                    "gps_map_yaw_from_enu": LaunchConfiguration(
                        "gps_map_yaw_from_enu"
                    ),
                }.items(),
            )
        ]
    )
