import os

import launch
import launch.actions
import launch.events
import lifecycle_msgs.msg

import launch_ros
import launch_ros.actions
import launch_ros.event_handlers
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    cloud_topic = LaunchConfiguration("cloud_topic", default="/velodyne_points")
    odom_topic = LaunchConfiguration("odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")

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
                "enable_map_odom_tf": False,
                "score_threshold": 10.0,
                "global_frame_id": "map",
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "enable_timer_publishing": True,
                "use_odom": False,
                "pose_publish_frequency": 30.0,
                "max_twist_prediction_dt": 0.35,
                "cloud_queue_depth": 5,
                "cloud_qos_reliability": "reliable",
            },
        ],
        remappings=[
            ("/cloud", cloud_topic),
            ("/odom", odom_topic),
            ("/imu", imu_topic),
        ],
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
            "cloud_topic",
            default_value="/velodyne_points",
            description="PointCloud2 topic published by velodyne_pointcloud.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/zed_front/zed_node_0/odom",
            description="Odometry topic used by lidar_localization_ros2.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/zed_front/zed_node_0/imu/data",
            description="IMU topic used when localization.yaml enables use_imu.",
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
    ld.add_action(TimerAction(period=3.0, actions=[to_inactive]))

    return ld
