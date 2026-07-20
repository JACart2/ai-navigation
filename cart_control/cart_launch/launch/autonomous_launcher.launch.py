import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_cart = os.environ.get("CART_ID", "james").strip().lower()
    if default_cart not in {"james", "madison"}:
        default_cart = "james"

    mola_autonomy_launch = os.path.join(
        get_package_share_directory("cart_launch"),
        "launch",
        "mola_autonomy.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart",
                default_value=default_cart,
                description="Cart-specific sensor configuration: james or madison.",
            ),
            DeclareLaunchArgument(
                "cart_config_path",
                default_value=os.path.join(
                    get_package_share_directory("cart_launch"),
                    "config",
                    f"cart_{default_cart}.yaml",
                ),
                description="Path to cart-specific YAML configuration.",
            ),
            DeclareLaunchArgument(
                "enable_aad",
                default_value="true",
                description="Enable anomaly logging.",
            ),
            DeclareLaunchArgument(
                "enable_aad_camera_capture",
                default_value="true",
                description=(
                    "Subscribe to raw cameras for bounded AAD stop-event context."
                ),
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
                    "cart": LaunchConfiguration("cart"),
                    "cart_config_path": LaunchConfiguration("cart_config_path"),
                    "enable_aad": LaunchConfiguration("enable_aad"),
                    "enable_aad_camera_capture": LaunchConfiguration(
                        "enable_aad_camera_capture"
                    ),
                    "launch_aad_node": LaunchConfiguration("launch_aad_node"),
                }.items(),
            ),
        ]
    )
