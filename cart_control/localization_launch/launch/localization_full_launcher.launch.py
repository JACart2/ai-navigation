from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import launch_ros
import launch_ros.actions
import launch_ros.events
import os


def generate_launch_description():

    # Launch the velodyne_driver node for VLP16
    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        parameters=[{"model": "VLP16"}],
    )

    # Include the velodyne_transform_node-VLP16-launch.py directly
    velodyne_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("velodyne_pointcloud"),
                "/launch/velodyne_transform_node-VLP16-launch.py",
            ]
        )
    )

    # Specify the new path to liosam.launch.py
    lio_sam_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "lio-sam.launch.py",
    )

    # Include the lidar_localization launch file using the new path
    # liosam_localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([lio_sam_launch_path]),
    #     launch_arguments={
    #         "localization_only": "true",
    #         "loop_closure_flag": "false",
    #         "static_map_path": "/home/jacart/jacart-project/dev_ws/src/maps/final_map_condensed_5-22.pcd",
    #     }.items(),
    # )

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
        PythonLaunchDescriptionSource([cameras_launch_path])
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            velodyne_driver_node,
            velodyne_transform_launch,
            lidar_localization_launch,
            cameras_launch,
            # liosam_localization_launch,
        ]
    )
