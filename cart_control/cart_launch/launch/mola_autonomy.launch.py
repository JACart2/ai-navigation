import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigation_maps_dir = os.path.join(
        get_package_share_directory("navigation"), "maps"
    )
    rviz_config = os.path.join(
        get_package_share_directory("cart_launch"),
        "rviz",
        "mola_localization.rviz",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    enable_motor = LaunchConfiguration("enable_motor")
    enable_aad = LaunchConfiguration("enable_aad")

    mola_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("localization_launch"),
                "/launch/mola_map_localization.launch.py",
            ]
        ),
        launch_arguments={
            # Disable MOLA's internal RViz so this top-level launch owns the
            # single RViz window for the real outdoor autonomy workflow.
            "use_rviz": "false",
            "map_file": LaunchConfiguration("map_file"),
            "lidar_topic": LaunchConfiguration("lidar_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
            "lidar_frame": LaunchConfiguration("lidar_frame"),
            "cart_name": LaunchConfiguration("cart_name"),
            "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
            "cart_config_path": LaunchConfiguration("cart_config_path"),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("navigation"), "/launch/navigation.launch.py"]
        ),
        launch_arguments={
            # Known-good James/MOLA outdoor route overlay from 2026-06-23.
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

    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("motor_control"), "/launch/motor.launch.py"]
        ),
        launch_arguments={
            "enable_aad": enable_aad,
        }.items(),
        condition=IfCondition(enable_motor),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description=(
                    "Open exactly one top-level RViz window using "
                    "mola_localization.rviz."
                ),
            ),
            DeclareLaunchArgument(
                "enable_motor",
                default_value="true",
                description=(
                    "Start motor_control by default for outdoor MOLA autonomy "
                    "testing; set enable_motor:=false for bench/dry-run testing."
                ),
            ),
            DeclareLaunchArgument(
                "enable_aad",
                default_value="false",
                description="Enable anomaly logging nodes.",
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
                "cart_name",
                default_value="james",
                description="Cart name used by MOLA localization setup.",
            ),
            DeclareLaunchArgument(
                "publish_odom_tf",
                default_value="true",
                description="Publish dynamic TF from MOLA odometry.",
            ),
            DeclareLaunchArgument(
                "cart_config_path",
                default_value="",
                description="Optional explicit path to cart-specific YAML config.",
            ),
            mola_localization_launch,
            navigation_launch,
            motor_launch,
            rviz_node,
        ]
    )
