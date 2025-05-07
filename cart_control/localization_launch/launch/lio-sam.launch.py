import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lio_sam_share_dir = get_package_share_directory("lio_sam")
    lio_sam_config = os.path.join(lio_sam_share_dir, "config", "params.yaml")

    declare_config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=lio_sam_config,
        description="Path to the LIO-SAM configuration file",
    )

    declare_loc_only_arg = DeclareLaunchArgument(
        "localization_only",
        default_value='false',
        description="Set to 'true' to run in localization-only mode",
    )

    declare_static_map_arg = DeclareLaunchArgument(
        "static_map_path",
        default_value="",
        description="Absolute path to the pre-built PCD map "
                    "(required if localization_only == true)",
    )

    config        = LaunchConfiguration("config_file")
    loc_only      = LaunchConfiguration("localization_only")
    static_map    = LaunchConfiguration("static_map_path")

    # When localization_only == true we also want loopClosureEnableFlag=false
    loop_closure_flag = PythonExpression(
        ["'false' if ", loc_only, " == 'true' else 'true'"]
    )

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
        parameters=[
            config,
            {
                # ←──── overrides land here
                "localization_only": loc_only,
                "static_map_path":   static_map,
                "loopClosureEnableFlag": loop_closure_flag,
            },
        ],
        arguments=["--ros-args", "--log-level", "map_optimization:=info"],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_config_arg,
            declare_loc_only_arg,
            declare_static_map_arg,
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
