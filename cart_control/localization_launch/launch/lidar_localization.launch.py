import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = launch.LaunchDescription()

    lidar_parent_frame = "base_link"
    lidar_child_frame = "velodyne"
    lidar_x = "1"
    lidar_y = "0"
    lidar_z = "1.9"
    lidar_roll = "0"
    lidar_pitch = "0"
    lidar_yaw = "0"

    lidar_tf = launch_ros.actions.Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            lidar_x,
            "--y",
            lidar_y,
            "--z",
            lidar_z,
            "--roll",
            lidar_roll,
            "--pitch",
            lidar_pitch,
            "--yaw",
            lidar_yaw,
            "--frame-id",
            lidar_parent_frame,
            "--child-frame-id",
            lidar_child_frame,
        ],
    )

    lidar_tf_fallback = launch_ros.actions.Node(
        name="lidar_tf_fallback",
        package="localization_launch",
        executable="tf_fallback_publisher",
        condition=IfCondition(LaunchConfiguration("enable_lidar_tf_fallback")),
        parameters=[
            {
                "parent_frame": lidar_parent_frame,
                "child_frame": lidar_child_frame,
                "x": float(lidar_x),
                "y": float(lidar_y),
                "z": float(lidar_z),
                "roll": float(lidar_roll),
                "pitch": float(lidar_pitch),
                "yaw": float(lidar_yaw),
                "check_period_s": 0.2,
                "missing_checks_before_fallback": 10,
            }
        ],
        output="screen",
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
        parameters=[localization_param_dir, {"use_odom": False}],
        remappings=[("/cloud", "/velodyne_points")],
        output="screen",
    )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_lidar_localization = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "enable_lidar_tf_fallback",
            default_value="false",
            description=(
                "Enable the Velodyne TF fallback publisher. Requires rebuilding "
                "localization_launch so tf_fallback_publisher is installed."
            ),
        )
    )

    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[
                lidar_localization,
                lidar_tf,
                lidar_tf_fallback,
                TimerAction(
                    period=2.0,
                    actions=[
                        launch.actions.LogInfo(msg="-- Configuring lidar_localization --"),
                        to_inactive,
                        launch.actions.LogInfo(
                            msg="-- Activating lidar_localization without waiting for /initial_map RViz subscriber --"
                        ),
                        activate_lidar_localization,
                    ],
                ),
            ],
        )
    )

    return ld
