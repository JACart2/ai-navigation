from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for MOLA mapping mode.
    Publishes only the essential topics needed for MOLA:
    - /velodyne_points (lidar data)
    - /zed_*/zed_node_*/imu/data (IMU data from ZED cameras)
    - TF frames (base_link -> velodyne)
    """
    
    cart_config_path = LaunchConfiguration("cart_config_path")

    # Launch the velodyne_driver node for VLP16
    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        parameters=[{"model": "VLP16"}],
        output="screen",
    )

    # Include the velodyne_transform_node for VLP16 to publish point clouds
    velodyne_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("velodyne_pointcloud"),
                "/launch/velodyne_transform_node-VLP16-launch.py",
            ]
        )
    )

    # Specify the path to cameras.launch.py for IMU data and camera TFs
    cameras_launch_path = os.path.join(
        get_package_share_directory("localization_launch"),
        "launch",
        "cameras.launch.py",
    )

    # Include the cameras launch file
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([cameras_launch_path]),
        launch_arguments={
            "cart_config_path": cart_config_path,
        }.items(),
    )

    # Static TF publisher for base_link to velodyne transform
    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_velodyne_tf",
        arguments=["0.5", "0", "1.75", "0", "0", "0", "base_link", "velodyne"],
        output="screen",
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart_config_path",
                default_value=os.path.join(
                    get_package_share_directory("cart_launch"), "config", "cart_james.yaml"
                ),
                description="Path to cart-specific YAML config (must contain zed_front_serial and zed_rear_serial)",
            ),
            velodyne_driver_node,
            velodyne_transform_launch,
            cameras_launch,
            static_tf_publisher,
        ]
    )
