from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    """Build launch actions after resolving cart selection arguments."""
    cart_name = LaunchConfiguration("cart_name").perform(context).strip().lower()
    cart_config_path_arg = LaunchConfiguration("cart_config_path").perform(context).strip()

    if cart_name not in {"james", "madison"}:
        raise RuntimeError(
            "Invalid cart_name. Use 'james' or 'madison', "
            f"got: '{cart_name}'."
        )

    cart_config_path = cart_config_path_arg or os.path.join(
        get_package_share_directory("cart_launch"),
        "config",
        f"cart_{cart_name}.yaml",
    )

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

    return [
        velodyne_driver_node,
        velodyne_transform_launch,
        cameras_launch,
        static_tf_publisher,
    ]


def generate_launch_description():
    """
    Launch file for MOLA mapping mode.
    Publishes only the essential topics needed for MOLA:
    - /velodyne_points (lidar data)
    - /zed_*/zed_node_*/imu/data (IMU data from ZED cameras)
    - TF frames (base_link -> velodyne)
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart_name",
                default_value="james",
                description="Cart profile to use: 'james' or 'madison'.",
            ),
            DeclareLaunchArgument(
                "cart_config_path",
                default_value="",
                description=(
                    "Optional path to cart-specific YAML config. "
                    "If empty, uses cart_<cart_name>.yaml from cart_launch/config."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
