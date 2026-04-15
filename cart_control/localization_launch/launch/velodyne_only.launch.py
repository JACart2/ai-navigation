from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    rpm = LaunchConfiguration("rpm")
    device_ip = LaunchConfiguration("device_ip")
    port = LaunchConfiguration("port")
    frame_id = LaunchConfiguration("frame_id")

    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        output="screen",
        parameters=[
            {
                "model": model,
                "rpm": ParameterValue(rpm, value_type=float),
                "device_ip": device_ip,
                "port": ParameterValue(port, value_type=int),
                "frame_id": frame_id,
            }
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="VLP16",
                description="Velodyne model (for example: VLP16, 32C, VLS128)",
            ),
            DeclareLaunchArgument(
                "rpm",
                default_value="600.0",
                description="LiDAR spin rate in RPM",
            ),
            DeclareLaunchArgument(
                "device_ip",
                default_value="192.168.1.201",
                description="Velodyne sensor IP address",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="2368",
                description="Velodyne UDP data port",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="velodyne",
                description="TF frame for outgoing packets",
            ),
            velodyne_driver_node,
            velodyne_transform_launch,
        ]
    )
