import os

import launch
import launch.actions
import launch.events
import lifecycle_msgs.msg

import launch_ros
import launch_ros.actions
import launch_ros.event_handlers
import launch_ros.events

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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

    localization_param_dir = LaunchConfiguration("localization_param_dir")

    lidar_localization = launch_ros.actions.LifecycleNode(
        name="lidar_localization",
        namespace="",
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        parameters=[localization_param_dir],
        remappings=[("/cloud", "/velodyne_points")],
        output="screen",
    )

    configure_lidar_localization = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_lidar_localization = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                launch.actions.LogInfo(msg="Activating lidar_localization"),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            lidar_localization
                        ),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "localization_param_dir",
            default_value=os.path.join(
                get_package_share_directory("localization_launch"),
                "param",
                "localization.yaml",
            ),
            description="Path to lidar_localization_ros2 parameter YAML",
        )
    )
    ld.add_action(activate_lidar_localization)
    ld.add_action(lidar_localization)
    ld.add_action(configure_lidar_localization)
    ld.add_action(pcl_pose_relay)
    ld.add_action(lidar_tf)
    ld.add_action(imu_tf)

    return ld
