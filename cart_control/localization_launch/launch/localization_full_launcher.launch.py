from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cart_config_path = LaunchConfiguration("cart_config_path")
    lidar_device_ip = LaunchConfiguration("lidar_device_ip")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")

    velodyne_driver_params = os.path.join(
        get_package_share_directory("velodyne_driver"),
        "config",
        "VLP16-velodyne_driver_node-params.yaml",
    )

    # Start the VLP16 driver with the upstream defaults, then override cart-specific connection settings.
    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        output="both",
        parameters=[
            velodyne_driver_params,
            {
                "device_ip": lidar_device_ip,
                "port": ParameterValue(lidar_port, value_type=int),
                "frame_id": lidar_frame_id,
            },
        ],
    )

    velodyne_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("velodyne_pointcloud"),
                "/launch/velodyne_transform_node-VLP16-launch.py",
            ]
        )
    )

    lidar_localization_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "lidar_localization.launch.py",
    )
    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_localization_launch_path])
    )

    cameras_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "cameras.launch.py",
    )
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([cameras_launch_path]),
        launch_arguments={
            "cart_config_path": cart_config_path,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart_config_path",
                description="Path to cart-specific YAML config (must contain zed_front_serial and zed_rear_serial)",
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
            velodyne_driver_node,
            velodyne_transform_launch,
            lidar_localization_launch,
            cameras_launch,
        ]
    )
