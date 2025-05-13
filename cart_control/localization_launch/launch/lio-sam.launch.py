import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Default config path
    lio_sam_share_dir = get_package_share_directory("lio_sam")
    lio_sam_config = os.path.join(lio_sam_share_dir, "config", "params.yaml")

    # Launch arguments
    declare_config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=lio_sam_config,
        description="Path to the LIO-SAM configuration file",
    )

    # Load launch config values
    config = LaunchConfiguration("config_file")

    # TF static transforms
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf",
        arguments=["1", "0", "1.9", "0", "0", "0", "base_link", "velodyne"],
        output="screen",
    )

    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        output="screen",
    )

    # LIO-SAM nodes
    image_projection = Node(
        package="lio_sam",
        executable="lio_sam_imageProjection",
        name="lio_sam_imageProjection",
        parameters=[
            config,
            {
                "imuTopic": "/zed_rear/zed_node_1/imu/data",
                "qos_overrides./velodyne_points.reliability": "reliable",
                "qos_overrides./zed_rear/zed_node/imu/data.reliability": "reliable",
                "qos_overrides./odometry/imu_incremental.reliability": "reliable",
            },
        ],
        remappings=[
            ("points", "/velodyne_points"),
            # ("imu/data", "/zed_rear/zed_node_1/imu/data"),
            ("odometry/imu", "/odometry/imu_incremental"),
        ],
        arguments=["--ros-args", "--log-level", "image_projection:=debug"],
        emulate_tty=True,
        output="screen",
    )

    imu_preintegration = Node(
        package="lio_sam",
        executable="lio_sam_imuPreintegration",
        name="lio_sam_imuPreintegration",
        namespace="lio_sam",
        parameters=[config],
        remappings=[("odometry/imu", "/odometry/imu_incremental")],
        arguments=["--ros-args", "--log-level", "warn"],
        output="screen",
    )

    feature_extraction = Node(
        package="lio_sam",
        executable="lio_sam_featureExtraction",
        name="lio_sam_featureExtraction",
        parameters=[config],
        arguments=["--ros-args", "--log-level", "feature_extraction:=info"],
        output="screen",
    )

    map_optimization = Node(
        package="lio_sam",
        executable="lio_sam_mapOptimization",
        name="lio_sam_mapOptimization",
        parameters=[config],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_config_arg,
            lidar_tf,
            imu_tf,
            TimerAction(
                period=3.0,
                actions=[
                    image_projection,
                    imu_preintegration,
                    TimerAction(
                        period=1.0,
                        actions=[feature_extraction, map_optimization],
                    ),
                ],
            ),
            ExecuteProcess(cmd=["ros2", "topic", "list"], output="screen"),
        ]
    )
