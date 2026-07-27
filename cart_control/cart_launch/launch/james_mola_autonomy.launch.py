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
                    "Start the conservative LiDAR-only MOLA auto-localization "
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
                }.items(),
            )
        ]
    )
