from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    zed_front_serial = LaunchConfiguration("zed_front_serial")
    zed_rear_serial = LaunchConfiguration("zed_rear_serial")

    # Specify the path to zed_multi_camera.launch.py
    zed_multi_camera_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "zed_multi_camera.launch.py",
    )

    # Include the zed_multi_camera launch file
    zed_multi_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([zed_multi_camera_launch_path]),
        # Pass launch arguments for cameras, models, serials, and TF configuration
        launch_arguments={
            "cam_names": "[zed_front, zed_rear]",  # Names of the cameras
            "cam_models": "[zed2i, zed2i]",  # Models of the cameras
            "cam_serials": PythonExpression([
                "[",
                zed_front_serial,
                ", ",
                zed_rear_serial,
                "]",
            ]),  # Serial numbers of the cameras
            "disable_tf": "False",  # Enable TF broadcasting
        }.items(),
    )

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
                "zed_front_serial",
                description="Serial number for zed_front camera",
            ),
            DeclareLaunchArgument(
                "zed_rear_serial",
                description="Serial number for zed_rear camera",
            ),
            zed_multi_camera_launch,
            multi_link_tf,
        ]
    )
