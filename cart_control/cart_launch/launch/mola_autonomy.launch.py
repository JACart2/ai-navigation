import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _source_or_share(source_path, package_name, *share_parts):
    if os.path.exists(source_path):
        return source_path
    return os.path.join(get_package_share_directory(package_name), *share_parts)


def generate_launch_description():
    navigation_maps_dir = _source_or_share(
        "/root/dev_ws/src/ai-navigation/navigation/maps",
        "navigation",
        "maps",
    )
    default_rviz_config = _source_or_share(
        (
            "/root/dev_ws/src/ai-navigation/cart_control/cart_launch/rviz/"
            "mola_localization.rviz"
        ),
        "cart_launch",
        "rviz",
        "mola_localization.rviz",
    )

    launch_rviz = LaunchConfiguration("launch_rviz")
    enable_motor = LaunchConfiguration("enable_motor")
    enable_aad = LaunchConfiguration("enable_aad")
    enable_mola_auto_localization = LaunchConfiguration(
        "enable_mola_auto_localization"
    )

    mola_auto_localization_params = PathJoinSubstitution(
        [
            FindPackageShare("localization_launch"),
            "param",
            "mola_auto_localization_supervisor.yaml",
        ]
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("localization_launch"),
                "/launch/velodyne_only.launch.py",
            ]
        ),
        launch_arguments={
            "cart": LaunchConfiguration("cart"),
            "cart_name": LaunchConfiguration("cart_name"),
            "cart_config_path": LaunchConfiguration("cart_config_path"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_velodyne")),
    )

    mola_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("localization_launch"),
                "/launch/mola_map_localization.launch.py",
            ]
        ),
        launch_arguments={
            "use_rviz": "false",
            "map_file": LaunchConfiguration("map_file"),
            "lidar_topic": LaunchConfiguration("lidar_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
            "lidar_frame": LaunchConfiguration("lidar_frame"),
            "lidar_x": LaunchConfiguration("lidar_x"),
            "lidar_y": LaunchConfiguration("lidar_y"),
            "lidar_z": LaunchConfiguration("lidar_z"),
            "lidar_yaw": LaunchConfiguration("lidar_yaw"),
            "lidar_pitch": LaunchConfiguration("lidar_pitch"),
            "lidar_roll": LaunchConfiguration("lidar_roll"),
            "cart": LaunchConfiguration("cart"),
            "cart_name": LaunchConfiguration("cart_name"),
            "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
            "cart_config_path": LaunchConfiguration("cart_config_path"),
        }.items(),
    )

    pcl_pose_relay = Node(
        package="localization_launch",
        executable="pcl_pose_relay",
        name="pcl_pose_relay",
        output="screen",
    )

    mola_auto_localization_supervisor = Node(
        package="localization_launch",
        executable="mola_auto_localization_supervisor",
        name="mola_auto_localization_supervisor",
        output="screen",
        parameters=[
            mola_auto_localization_params,
            {
                "cloud_topic": LaunchConfiguration("lidar_topic"),
                "mola_pose_topic": "/lidar_odometry/pose",
            },
        ],
        condition=IfCondition(enable_mola_auto_localization),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("navigation"), "/launch/navigation.launch.py"]
        ),
        launch_arguments={
            "graph_dir": LaunchConfiguration("graph_dir"),
            "graph_file": LaunchConfiguration("graph_file"),
            "graph_coordinate_format": LaunchConfiguration(
                "graph_coordinate_format"
            ),
            "calibration_config_dir": LaunchConfiguration(
                "calibration_config_dir"
            ),
            "calibration_config_file": LaunchConfiguration(
                "calibration_config_file"
            ),
            "enable_aad": enable_aad,
        }.items(),
    )

    rviz_node = ExecuteProcess(
        cmd=["rviz2", "-d", LaunchConfiguration("rviz_config")],
        name="rviz2_mola_localization",
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    rosbridge_cleanup = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "pkill -f '[r]osbridge_websocket' || true; "
                "pkill -f '[r]osapi_node' || true; "
                "pkill -f '[r]osbridge_server rosbridge_websocket_launch.xml' || true; "
                "sleep 1"
            ),
        ],
        name="cleanup_stale_rosbridge",
        output="screen",
    )

    rosbridge_launch = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    (
                        "source /opt/ros/jazzy/setup.bash && "
                        "source /root/dev_ws/install/setup.bash && "
                        "exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
                    ),
                ],
                name="rosbridge_manual_launch",
                output="screen",
            )
        ],
    )

    motor_launch = Node(
        package="motor_control",
        executable="motor_endpoint",
        name="motor_endpoint",
        output="screen",
        parameters=[
            {
                "arduino_port": LaunchConfiguration("motor_port"),
                "baudrate": ParameterValue(
                    LaunchConfiguration("motor_baudrate"),
                    value_type=int,
                ),
            }
        ],
        condition=IfCondition(enable_motor),
    )

    wait_for_pcl_pose = ExecuteProcess(
        cmd=["ros2", "topic", "echo", "/pcl_pose", "--once"],
        name="wait_for_pcl_pose",
        output="screen",
    )

    delayed_route_stack = TimerAction(
        period=LaunchConfiguration("navigation_start_delay"),
        actions=[
            LogInfo(msg="Starting navigation route stack."),
            navigation_launch,
            motor_launch,
        ],
    )

    start_navigation_after_pcl_pose = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_pcl_pose,
            on_exit=[
                LogInfo(
                    msg=(
                        "Observed /pcl_pose; waiting for RViz subscribers "
                        "before starting navigation."
                    )
                ),
                delayed_route_stack,
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart",
                default_value="madison",
                description=(
                    "Cart used to select cart-specific localization and "
                    "sensor configuration: james or madison."
                ),
            ),
            DeclareLaunchArgument(
                "cart_name",
                default_value="",
                description=(
                    "Legacy alias for cart. Prefer cart:=james or "
                    "cart:=madison."
                ),
            ),
            DeclareLaunchArgument(
                "cart_config_path",
                default_value="",
                description="Optional explicit path to cart-specific YAML config.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz preset to open when use_rviz is true.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Open the JACart MOLA RViz preset window.",
            ),
            DeclareLaunchArgument(
                "start_velodyne",
                default_value="true",
                description=(
                    "Start the Velodyne driver/transform stack that publishes "
                    "/velodyne_points."
                ),
            ),
            DeclareLaunchArgument(
                "navigation_start_delay",
                default_value="5.0",
                description=(
                    "Seconds to wait after /pcl_pose before starting "
                    "navigation, giving RViz time to subscribe."
                ),
            ),
            DeclareLaunchArgument(
                "enable_motor",
                default_value="true",
                description=(
                    "Start motor_control by default; set enable_motor:=false "
                    "for safe no-motor testing."
                ),
            ),
            DeclareLaunchArgument(
                "motor_port",
                default_value="/dev/ttyUSB0",
                description="Serial device used by motor_endpoint.",
            ),
            DeclareLaunchArgument(
                "motor_baudrate",
                default_value="57600",
                description="Serial baud rate used by motor_endpoint.",
            ),
            DeclareLaunchArgument(
                "enable_aad",
                default_value="false",
                description="Enable anomaly logging nodes.",
            ),
            DeclareLaunchArgument(
                "enable_mola_auto_localization",
                default_value="false",
                description=(
                    "Start the conservative LiDAR-only MOLA auto-localization "
                    "supervisor."
                ),
            ),
            DeclareLaunchArgument(
                "graph_dir",
                default_value=navigation_maps_dir,
                description="Directory containing navigation graph files.",
            ),
            DeclareLaunchArgument(
                "graph_file",
                default_value="main_shift3_gps.gml",
                description="Navigation graph file.",
            ),
            DeclareLaunchArgument(
                "graph_coordinate_format",
                default_value="gps",
                description='Graph coordinate format: "gps" or "ros".',
            ),
            DeclareLaunchArgument(
                "calibration_config_dir",
                default_value=navigation_maps_dir,
                description="Directory containing landmark calibration YAML.",
            ),
            DeclareLaunchArgument(
                "calibration_config_file",
                default_value="with_gps2_adjusted_route.yaml",
                description="Landmark calibration YAML file.",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value="/root/dev_ws/maps/with_gps2.mm",
                description="Georeferenced MOLA map file.",
            ),
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/velodyne_points",
                description="Live LiDAR PointCloud2 topic.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Robot base frame used by MOLA.",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="velodyne",
                description="LiDAR frame for the static sensor transform.",
            ),
            DeclareLaunchArgument(
                "lidar_x",
                default_value="0.5",
                description="Shared/default LiDAR x offset when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_y",
                default_value="0.0",
                description="Shared/default LiDAR y offset when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_z",
                default_value="1.75",
                description="Shared/default LiDAR z offset when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_yaw",
                default_value="0.0",
                description="Shared/default LiDAR yaw when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_pitch",
                default_value="0.0",
                description="Shared/default LiDAR pitch when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "lidar_roll",
                default_value="0.0",
                description="Shared/default LiDAR roll when cart YAML has no lidar_tf.",
            ),
            DeclareLaunchArgument(
                "publish_odom_tf",
                default_value="true",
                description="Publish dynamic TF from MOLA odometry.",
            ),
            velodyne_launch,
            mola_localization_launch,
            pcl_pose_relay,
            mola_auto_localization_supervisor,
            rviz_node,
            rosbridge_cleanup,
            rosbridge_launch,
            wait_for_pcl_pose,
            start_navigation_after_pcl_pose,
        ]
    )
