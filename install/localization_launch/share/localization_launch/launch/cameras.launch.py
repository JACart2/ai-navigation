from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    cart_config_path = LaunchConfiguration("cart_config_path")

    # Specify the path to zed_multi_camera.launch.py
    zed_multi_camera_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "zed_multi_camera.launch.py",
    )

    def _include_zed_multi_camera(context, *args, **kwargs):
        cfg_path = perform_substitutions(context, [cart_config_path]).strip()
        if not cfg_path:
            raise RuntimeError("Required launch argument 'cart_config_path' is empty")

        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}

        zed_front_serial = str(cfg.get("zed_front_serial", "")).strip()
        zed_rear_serial = str(cfg.get("zed_rear_serial", "")).strip()

        if not zed_front_serial or not zed_rear_serial:
            raise RuntimeError(
                "cart_config_path YAML must define non-empty 'zed_front_serial' and 'zed_rear_serial'"
            )

        # Include the zed_multi_camera launch file
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([zed_multi_camera_launch_path]),
                # Pass launch arguments for cameras, models, serials, and TF configuration
                launch_arguments={
                    "cam_names": "[zed_front, zed_rear]",  # Names of the cameras
                    "cam_models": "[zed2i, zed2i]",  # Models of the cameras
                    "cam_serials": f"[{zed_front_serial}, {zed_rear_serial}]",  # Serial numbers of the cameras
                    "disable_tf": "False",  # Enable TF broadcasting
                }.items(),
            )
        ]

    zed_multi_camera_launch = OpaqueFunction(function=_include_zed_multi_camera)

    # Static transform for the reference link (zed_multi_link) to base_link
    multi_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="multi_link_tf",
        arguments=[
            "1.0",  # X position (adjust as needed)
            "0.0",  # Y position (adjust as needed)
            "1.6",  # Z position (height of cameras above ground)
            "0.0",  # Roll (rotation around X-axis)
            "0.0",  # Pitch (rotation around Y-axis)
            "0.0",  # Yaw (rotation around Z-axis)
            "base_link",  # Parent frame (golf cart base)
            "zed_front_camera_link",  # Child frame (reference link for cameras)
        ],
        output="screen",
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart_config_path",
                description="Path to cart-specific YAML config (must contain zed_front_serial and zed_rear_serial)",
            ),
            zed_multi_camera_launch,
            multi_link_tf,
        ]
    )
