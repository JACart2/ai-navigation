
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    lidar_tf = Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["1", "0", "1.9", "0", "0", "0", "1", "base_link", "velodyne"],
    )

    imu_tf = Node(
        name="imu_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "imu_link"],
    )

    # Load the map right away
    map_mm = LaunchConfiguration('MOLA_LOAD_MM', default='/tmp/my_map.mm')
    map_sm = LaunchConfiguration('MOLA_LOAD_SM', default='/tmp/my_map.simplemap')

    # MOLA-LO launch in localization-only mode with map loading
    mola_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("mola_lidar_odometry"),
                "/ros2-launchs/ros2-lidar-odometry.launch.py"
            ]
        ),
        launch_arguments={
            "start_mapping_enabled": 'false',
            "start_active": 'false',
            "lidar_topic_name": "/velodyne_points",
            # 'MOLA_LOAD_MM': map_mm,
            # 'MOLA_LOAD_SM': map_sm,
        }.items(),
    )

    return LaunchDescription([
        lidar_tf,
        imu_tf,
        mola_localization
    ])

