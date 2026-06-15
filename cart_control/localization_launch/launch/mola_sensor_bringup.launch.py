import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Safe bringup for Velodyne LiDAR + MOLA odometry/visualization only.

    This launch file starts the existing Velodyne point cloud pipeline and then
    starts the existing MOLA odometry wrapper. It does not enable mapping,
    simplemap generation, cameras, old lidar_localization, navigation, or any
    TF ownership changes.
    """

    lidar_topic_name = LaunchConfiguration("lidar_topic_name")
    lidar_topic_type = LaunchConfiguration("lidar_topic_type")
    start_active = LaunchConfiguration("start_active")

    model = LaunchConfiguration("model")
    rpm = LaunchConfiguration("rpm")
    device_ip = LaunchConfiguration("device_ip")
    port = LaunchConfiguration("port")
    frame_id = LaunchConfiguration("frame_id")

    localization_launch_share = get_package_share_directory("localization_launch")

    velodyne_launch_path = os.path.join(
        localization_launch_share,
        "launch",
        "velodyne_only.launch.py",
    )
    mola_lidar_odometry_launch_path = os.path.join(
        localization_launch_share,
        "launch",
        "mola_lidar_odometry.launch.py",
    )

    velodyne_only_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_launch_path),
        launch_arguments={
            "model": model,
            "rpm": rpm,
            "device_ip": device_ip,
            "port": port,
            "frame_id": frame_id,
        }.items(),
    )

    mola_lidar_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mola_lidar_odometry_launch_path),
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
            DeclareLaunchArgument(
                "model",
                default_value="VLP16",
                description="Velodyne model forwarded to velodyne_only.launch.py.",
            ),
            DeclareLaunchArgument(
                "rpm",
                default_value="600.0",
                description="Velodyne spin rate in RPM.",
            ),
            DeclareLaunchArgument(
                "device_ip",
                default_value="192.168.1.201",
                description="Velodyne sensor IP address.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="2368",
                description="Velodyne UDP data port.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="velodyne",
                description="TF frame for outgoing Velodyne packets.",
            ),
            velodyne_only_launch,
            mola_lidar_odometry_launch,
        ]
    )
