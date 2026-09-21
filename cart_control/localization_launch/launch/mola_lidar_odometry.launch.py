import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Safe wrapper for MOLA LiDAR odometry/visualization.

    This intentionally launches only MOLA's LiDAR odometry entry point using the
    current Velodyne point cloud topic. It is not a full replacement for the
    existing JACart localization system yet, and it does not enable mapping,
    simplemap generation, map saving, or any existing localization/TF changes.
    """

    lidar_topic_name = LaunchConfiguration("lidar_topic_name")
    lidar_topic_type = LaunchConfiguration("lidar_topic_type")
    start_active = LaunchConfiguration("start_active")

    mola_launch_path = os.path.join(
        get_package_share_directory("mola_lidar_odometry"),
        "ros2-launchs",
        "ros2-lidar-odometry.launch.py",
    )

    mola_lidar_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mola_launch_path),
        launch_arguments={
            "lidar_topic_name": lidar_topic_name,
            "lidar_topic_type": lidar_topic_type,
            "start_active": start_active,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_topic_name",
                default_value="/velodyne_points",
                description="Input LiDAR topic for MOLA odometry.",
            ),
            DeclareLaunchArgument(
                "lidar_topic_type",
                default_value="PointCloud2",
                description="Input LiDAR message type. Must be exactly PointCloud2.",
            ),
            DeclareLaunchArgument(
                "start_active",
                default_value="True",
                description="Start MOLA odometry active by default.",
            ),
            mola_lidar_odometry_launch,
        ]
    )
