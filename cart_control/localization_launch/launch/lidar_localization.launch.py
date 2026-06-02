import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = launch.LaunchDescription()

    lidar_tf = launch_ros.actions.Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "1",
            "--y",
            "0",
            "--z",
            "1.9",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "velodyne",
        ],
    )

    imu_tf = launch_ros.actions.Node(
        name="imu_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "imu_link",
        ],
    )

    map_tf = launch_ros.actions.Node(
        name="map_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "base_link",
        ],
    )

    # Set the default path directly to the specific YAML file location
    localization_param_dir = LaunchConfiguration(
        "localization_param_dir",
        default=os.path.join(
            get_package_share_directory("localization_launch"),
            "param",
            "localization.yaml",
        ),
    )

    lidar_localization = launch_ros.actions.LifecycleNode(
        name="lidar_localization",
        namespace="",
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        parameters=[localization_param_dir],
        remappings=[
            ("/cloud", "/velodyne_points"),
            ("/odom", "/zed_front/zed_node_0/odom"),
            ("/imu", "/zed/zed_node/imu/data"),
        ],
        output="screen",
    )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    wait_for_initial_map_rviz = Node(
        package="navigation",
        executable="wait_for_rviz_subscriber",
        name="wait_for_initial_map_rviz",
        output="screen",
        parameters=[
            {
                "topic": "/initial_map",
                "message_type": "sensor_msgs/msg/PointCloud2",
                "poll_period_s": 0.5,
            }
        ],
    )

    activate_lidar_localization = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    activate_on_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_initial_map_rviz,
            on_exit=[
                launch.actions.LogInfo(
                    msg="-- RViz subscribed to /initial_map; activating lidar_localization --"
                ),
                activate_lidar_localization,
            ],
        )
    )

    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[
                lidar_localization,
                map_tf,
                lidar_tf,
                imu_tf,
                TimerAction(
                    period=2.0,
                    actions=[
                        launch.actions.LogInfo(msg="-- Configuring lidar_localization --"),
                        to_inactive,
                        wait_for_initial_map_rviz,
                        activate_on_rviz,
                    ],
                ),
            ],
        )
    )

    return ld
