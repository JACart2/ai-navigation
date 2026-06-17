"""
Localization pipeline that feeds /velodyne_points directly into NDT localization,
bypassing the packet downsample filter and the TF fallback node.

Usage (real sensors):
  ros2 launch localization_launch localization_direct_launcher.launch.py \
    cart_config_path:=/path/to/cart.yaml

Usage (bag file):
  ros2 launch localization_launch localization_direct_launcher.launch.py \
    bag_file:=/path/to/bag.mcap
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    cart_config_path = LaunchConfiguration("cart_config_path")
    lidar_device_ip = LaunchConfiguration("lidar_device_ip")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    bag_file = LaunchConfiguration("bag_file")

    bag_mode = PythonExpression(["'true' if '", bag_file, "' != '' else 'false'"])

    velodyne_driver_params = os.path.join(
        get_package_share_directory("velodyne_driver"),
        "config",
        "VLP16-velodyne_driver_node-params.yaml",
    )
    velodyne_pointcloud_share = get_package_share_directory("velodyne_pointcloud")
    velodyne_transform_params_file = os.path.join(
        velodyne_pointcloud_share,
        "config",
        "VLP16-velodyne_transform_node-params.yaml",
    )
    with open(velodyne_transform_params_file, "r") as f:
        velodyne_transform_params = yaml.safe_load(f)["velodyne_transform_node"][
            "ros__parameters"
        ]
    velodyne_transform_params["calibration"] = os.path.join(
        velodyne_pointcloud_share, "params", "VLP16db.yaml"
    )
    velodyne_transform_params["max_range"] = 90.0
    velodyne_transform_params["organize_cloud"] = False

    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        output="both",
        condition=UnlessCondition(bag_mode),
        parameters=[
            velodyne_driver_params,
            {
                "device_ip": lidar_device_ip,
                "port": ParameterValue(lidar_port, value_type=int),
                "frame_id": lidar_frame_id,
                "model": "VLP16",
                "rpm": 600.0,
            },
        ],
    )

    bag_play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", bag_file,
            "--topics", "/velodyne_packets", "/tf_static",
            "--clock", "--loop",
        ],
        condition=IfCondition(bag_mode),
        output="screen",
    )

    # Convert packets directly to PointCloud2 with no downsampling.
    velodyne_transform_node = Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        name="velodyne_transform_node",
        output="both",
        parameters=[velodyne_transform_params, {"use_sim_time": bag_mode}],
    )

    lidar_localization_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "lidar_localization.launch.py",
    )
    # Feed /velodyne_points directly; lidar_localization's default cloud_topic is
    # /velodyne_points_stable, so we override it here.
    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_localization_launch_path]),
        launch_arguments={
            "cloud_topic": "/velodyne_points",
            "use_sim_time": bag_mode,
        }.items(),
    )

    cameras_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "cameras.launch.py",
    )
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([cameras_launch_path]),
        condition=UnlessCondition(bag_mode),
        launch_arguments={
            "cart_config_path": cart_config_path,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_file",
                default_value="",
                description=(
                    "Path to a bag file. When set, plays the bag instead of starting "
                    "the Velodyne driver and cameras, and enables use_sim_time."
                ),
            ),
            DeclareLaunchArgument(
                "cart_config_path",
                default_value="",
                description=(
                    "Path to cart-specific YAML config (must contain zed_front_serial "
                    "and zed_rear_serial). Required when not using a bag file."
                ),
            ),
            DeclareLaunchArgument(
                "lidar_device_ip",
                default_value="192.168.1.201",
                description="Velodyne lidar IPv4 address.",
            ),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="2368",
                description="UDP port used by the Velodyne lidar.",
            ),
            DeclareLaunchArgument(
                "lidar_frame_id",
                default_value="velodyne",
                description="Frame id assigned to Velodyne packets and point clouds.",
            ),
            bag_play,
            velodyne_driver_node,
            velodyne_transform_node,
            lidar_localization_launch,
            cameras_launch,
        ]
    )
