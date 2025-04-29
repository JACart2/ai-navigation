import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get path to the shared config directory
    lio_sam_share_dir = get_package_share_directory("lio_sam")
    lio_sam_config = os.path.join(lio_sam_share_dir, "config", "params.yaml")

    declare_config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=lio_sam_config,
        description="Path to LIO-SAM configuration file",
    )

    config = LaunchConfiguration("config_file")

    # Static TFs
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

    # LIO-SAM components
    image_projection = Node(
        package="lio_sam",
        executable="lio_sam_imageProjection",
        name="lio_sam_imageProjection",
        parameters=[
            config,
            {
                "qos_overrides./velodyne_points.reliability": "reliable",
                "qos_overrides./zed_rear/zed_node/imu/data.reliability": "reliable",
                "qos_overrides./odometry/imu_incremental.reliability": "reliable",
            },
        ],
        remappings=[
            ("points", "/velodyne_points"),
            ("imu/data", "/zed_rear/zed_node/imu/data"),
            ("odometry/imu", "/odometry/imu_incremental"),
        ],
        arguments=["--ros-args", "--log-level", "image_projection:=debug"],
        emulate_tty=True,
        output="screen",
    )

    imu_preintegration = Node(
        package="lio_sam",
        executable="lio_sam_imuPreintegration",
        name="lio_sam_imuPreintegration",  # Ensure this is UNIQUE
        namespace="lio_sam",  # Add this line
        parameters=[config],
        remappings=[
            ("odometry/imu", "/odometry/imu_incremental"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],  # Reduced verbosity
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
        arguments=["--ros-args", "--log-level", "map_optimization:=info"],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_config_arg,
            lidar_tf,
            imu_tf,
            TimerAction(
                period=3.0,  # Increased delay for TF stabilization
                actions=[
                    image_projection,
                    imu_preintegration,
                    TimerAction(
                        period=1.0, actions=[feature_extraction, map_optimization]
                    ),
                ],
            ),
            ExecuteProcess(cmd=["ros2", "topic", "list"], output="screen"),
        ]
    )
