from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import launch_ros.actions
import launch_ros.events
import os


def generate_launch_description():
    cart_config_path = LaunchConfiguration("cart_config_path")

    # Launch the velodyne_driver node for VLP16
    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        parameters=[{"model": "VLP16"}],
    )

    # Launch the velodyne_transform_node directly so our params actually apply
    velodyne_transform_node = Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        name="velodyne_transform_node",
        output="screen",
        parameters=[{
            "calibration": "/opt/ros/jazzy/share/velodyne_pointcloud/params/VLP16db.yaml",
            "model": "VLP16",
            "min_range": 1.0,
            "max_range": 50.0,
            "organize_cloud": False,
            "fixed_frame": "",
            "target_frame": "",
            "use_sim_time": False,
            "view_direction": 0.0,
            "view_width": 6.283185307179586,
        }],
    )

    # Specify the new path to lidar_localization.launch.py
    lidar_localization_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "lidar_localization.launch.py",
    )

    # Include the lidar_localization launch file using the new path
    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_localization_launch_path])
    )

    # Specify the path to cameras.launch.py
    cameras_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "cameras.launch.py",
    )

    # Include the cameras launch file
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([cameras_launch_path]),
        launch_arguments={
            "cart_config_path": cart_config_path,
        }.items(),
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart_config_path",
                description="Path to cart-specific YAML config (must contain zed_front_serial and zed_rear_serial)",
            ),
            velodyne_driver_node,
            velodyne_transform_node,
            lidar_localization_launch,
            cameras_launch,
            # liosam_localization_launch,
        ]
    )
