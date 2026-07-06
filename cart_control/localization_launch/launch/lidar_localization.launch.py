import launch

import launch_ros
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    cloud_topic = LaunchConfiguration("cloud_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    lidar_tf = launch_ros.actions.Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["1", "0", "1.9", "0", "0", "0", "1", "base_link", "velodyne"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    imu_tf = launch_ros.actions.Node(
        name="imu_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "imu_link"],
    )

    pcl_pose_relay = launch_ros.actions.Node(
        name="pcl_pose_relay",
        package="localization_launch",
        executable="pcl_pose_relay",
        output="screen",
    )

    ld.add_action(pcl_pose_relay)
    ld.add_action(lidar_tf)
    ld.add_action(imu_tf)

    return ld
