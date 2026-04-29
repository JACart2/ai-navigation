import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = launch.LaunchDescription()

    enable_lidar = LaunchConfiguration("enable_lidar", default="true")
    enable_radar = LaunchConfiguration("enable_radar", default="true")

    declare_enable_lidar = DeclareLaunchArgument(
        "enable_lidar",
        default_value="true",
        description="Enable lidar localization and lidar transform",
    )
    declare_enable_radar = DeclareLaunchArgument(
        "enable_radar",
        default_value="true",
        description="Enable radar transform and radar obstacle pipeline",
    )

    lidar_tf = launch_ros.actions.Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["1", "0", "1.9", "0", "0", "0", "1", "base_link", "velodyne"],
        condition=IfCondition(enable_lidar),
    )

    imu_tf = launch_ros.actions.Node(
        name="imu_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "imu_link"],
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

    radar_x = LaunchConfiguration("radar_x", default="1.0")
    radar_y = LaunchConfiguration("radar_y", default="0.0")
    radar_z = LaunchConfiguration("radar_z", default="0.9")
    radar_roll = LaunchConfiguration("radar_roll", default="0.0")
    radar_pitch = LaunchConfiguration("radar_pitch", default="0.0")
    radar_yaw = LaunchConfiguration("radar_yaw", default="0.0")
    radar_parent_frame = LaunchConfiguration("radar_parent_frame", default="base_link")
    radar_frame = LaunchConfiguration("radar_frame", default="ti_mmwave_0")

    radar_tf = launch_ros.actions.Node(
        name="radar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            radar_x,
            radar_y,
            radar_z,
            radar_roll,
            radar_pitch,
            radar_yaw,
            radar_parent_frame,
            radar_frame,
        ],
        condition=IfCondition(enable_radar),
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
        condition=IfCondition(enable_lidar),
    )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state="unconfigured",
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            lidar_localization
                        ),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
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

    ld.add_action(declare_enable_lidar)
    ld.add_action(declare_enable_radar)
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    ld.add_action(lidar_localization)
    ld.add_action(lidar_tf)
    ld.add_action(imu_tf)
    ld.add_action(radar_tf)
    ld.add_action(to_inactive)

    return ld
