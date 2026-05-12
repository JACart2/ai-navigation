import launch

import launch_ros
import launch_ros.actions

from launch import LaunchDescription


def generate_launch_description():

    ld = launch.LaunchDescription()

    lidar_tf = launch_ros.actions.Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["1", "0", "1.9", "0", "0", "0", "1", "base_link", "velodyne"],
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
