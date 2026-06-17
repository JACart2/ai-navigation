import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

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
        parameters=[
            localization_param_dir,
            {
                "score_threshold": 10.0,
                "global_frame_id": "map",
                "base_frame_id": "base_link",
                "enable_timer_publishing": True,
                "pose_publish_frequency": 30.0,
                "max_twist_prediction_dt": 0.35,
                "cloud_queue_depth": 5,
                "cloud_qos_reliability": "best_effort",
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("/cloud", cloud_topic),
        ],
        output="screen",
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

    ld.add_action(
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/velodyne_points_stable",
            description="Stabilized PointCloud2 topic published by the Velodyne TF fallback node.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock (set true when replaying a bag with --clock).",
        )
    )
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    ld.add_action(lidar_localization)
    ld.add_action(lidar_tf)
    ld.add_action(TimerAction(period=3.0, actions=[to_inactive]))

    return ld
