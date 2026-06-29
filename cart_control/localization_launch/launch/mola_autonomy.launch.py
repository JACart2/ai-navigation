import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _source_or_share(source_path, package_name, *share_parts):
    if os.path.exists(source_path):
        return source_path
    return os.path.join(get_package_share_directory(package_name), *share_parts)


def generate_launch_description():
    cart_launch_path = os.path.join(
        get_package_share_directory("cart_launch"),
        "launch",
        "mola_autonomy.launch.py",
    )
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

    forwarded_args = {
        "cart": LaunchConfiguration("cart"),
        "cart_name": LaunchConfiguration("cart_name"),
        "cart_config_path": LaunchConfiguration("cart_config_path"),
        "rviz_config": LaunchConfiguration("rviz_config"),
        "launch_rviz": LaunchConfiguration("use_rviz"),
        "start_velodyne": LaunchConfiguration("start_velodyne"),
        "navigation_start_delay": LaunchConfiguration("navigation_start_delay"),
        "enable_motor": LaunchConfiguration("enable_motor"),
        "enable_mola_auto_localization": LaunchConfiguration(
            "enable_mola_auto_localization"
        ),
        "motor_port": LaunchConfiguration("motor_port"),
        "motor_baudrate": LaunchConfiguration("motor_baudrate"),
        "enable_aad": LaunchConfiguration("enable_aad"),
        "graph_dir": LaunchConfiguration("graph_dir"),
        "graph_file": LaunchConfiguration("graph_file"),
        "graph_coordinate_format": LaunchConfiguration("graph_coordinate_format"),
        "calibration_config_dir": LaunchConfiguration("calibration_config_dir"),
        "calibration_config_file": LaunchConfiguration("calibration_config_file"),
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
        "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart",
                default_value="madison",
                description="Cart used to select cart-specific configuration.",
            ),
            DeclareLaunchArgument(
                "cart_name",
                default_value="",
                description="Legacy alias for cart.",
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
                "use_rviz",
                default_value="true",
                description="Open exactly one RViz window.",
            ),
            DeclareLaunchArgument(
                "start_velodyne",
                default_value="true",
                description="Start the Velodyne driver/transform stack.",
            ),
            DeclareLaunchArgument(
                "navigation_start_delay",
                default_value="5.0",
                description="Seconds to wait after /pcl_pose before navigation.",
            ),
            DeclareLaunchArgument(
                "enable_motor",
                default_value="true",
                description="Start motor_control unless set to false.",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cart_launch_path),
                launch_arguments=forwarded_args.items(),
            ),
        ]
    )
