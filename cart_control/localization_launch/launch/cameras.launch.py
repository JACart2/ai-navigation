from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    auto_enable_obj_det = LaunchConfiguration("auto_enable_obj_det")
    obj_det_start_delay_sec = LaunchConfiguration("obj_det_start_delay_sec")
    obj_det_rear_start_delay_sec = LaunchConfiguration("obj_det_rear_start_delay_sec")

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
            "cam_serials": "[37963597, 31061594]",  # Serial numbers of the cameras
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

    enable_front_obj_det = TimerAction(
        period=obj_det_start_delay_sec,
        actions=[
            ExecuteProcess(
                condition=IfCondition(auto_enable_obj_det),
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/zed_front/zed_node_0/enable_obj_det",
                    "std_srvs/srv/SetBool",
                    "{data: true}",
                ],
                output="screen",
            )
        ],
    )

    enable_rear_obj_det = TimerAction(
        period=obj_det_rear_start_delay_sec,
        actions=[
            ExecuteProcess(
                condition=IfCondition(auto_enable_obj_det),
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/zed_rear/zed_node_1/enable_obj_det",
                    "std_srvs/srv/SetBool",
                    "{data: true}",
                ],
                output="screen",
            )
        ],
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auto_enable_obj_det",
                default_value="true",
                description="If true, automatically start ZED object detection via service calls.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "obj_det_start_delay_sec",
                default_value="10.0",
                description="Delay before auto-calling front camera object detection service.",
            ),
            DeclareLaunchArgument(
                "obj_det_rear_start_delay_sec",
                default_value="12.0",
                description="Delay before auto-calling rear camera object detection service.",
            ),
            zed_multi_camera_launch,
            multi_link_tf,
            enable_front_obj_det,
            enable_rear_obj_det,
        ]
    )
