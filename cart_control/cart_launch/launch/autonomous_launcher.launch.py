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
                "cart_config_path",
                default_value=os.path.join(
                    get_package_share_directory("cart_launch"),
                    "config",
                    "cart_james.yaml",
                ),
                description="Path to cart-specific YAML configuration.",
            ),
            DeclareLaunchArgument(
                "enable_aad",
                default_value="true",
                description="Enable anomaly logging.",
            ),
            DeclareLaunchArgument(
                "launch_aad_node",
                default_value="false",
                description=(
                    "Launch anomaly detection in this container. Leave false when "
                    "using the dedicated anomaly_detection compose service."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mola_autonomy_launch),
                launch_arguments={
                    "cart": "james",
                    "cart_config_path": LaunchConfiguration("cart_config_path"),
                    "enable_aad": LaunchConfiguration("enable_aad"),
                    "launch_aad_node": LaunchConfiguration("launch_aad_node"),
                }.items(),
            ),
        ]
    )
