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

    # Convert raw Velodyne packets into the /velodyne_points PointCloud2 topic
    # consumed by RViz, localization, and obstacle conversion.
    velodyne_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("velodyne_pointcloud"),
                "/launch/velodyne_transform_node-VLP16-launch.py",
            ]
        )
    )

    velodyne_pointcloud_tf_fallback = Node(
        package="localization_launch",
        executable="velodyne_pointcloud_tf_fallback",
        name="velodyne_pointcloud_tf_fallback",
        parameters=[
            {
                "input_topic": "/velodyne_points",
                "output_topic": "/velodyne_points_stable",
                "target_frame": "map",
                "parent_frame": "base_link",
                "child_frame": "velodyne",
                "x": 1.0,
                "y": 0.0,
                "z": 1.9,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "prefer_fallback_lidar_tf": True,
                "republish_original_if_possible": True,
            }
        ],
        output="screen",
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
            velodyne_transform_launch,
            velodyne_pointcloud_tf_fallback,
            lidar_localization_launch,
            cameras_launch,
            # liosam_localization_launch,
        ]
    )
