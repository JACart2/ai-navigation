import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _default_imu_qw(qx: float, qy: float, qz: float) -> float:
    qw2 = 1.0 - (qx * qx + qy * qy + qz * qz)
    return math.sqrt(qw2) if qw2 > 0.0 else 0.0


def generate_launch_description():
    # Velodyne args
    model = LaunchConfiguration("model")
    rpm = LaunchConfiguration("rpm")
    device_ip = LaunchConfiguration("device_ip")
    port = LaunchConfiguration("port")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")

    # Lidar TF args (base_link -> lidar_frame_id)
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_qx = LaunchConfiguration("lidar_qx")
    lidar_qy = LaunchConfiguration("lidar_qy")
    lidar_qz = LaunchConfiguration("lidar_qz")
    lidar_qw = LaunchConfiguration("lidar_qw")

    # IMU TF args (base_link -> imu_link)
    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_qx = LaunchConfiguration("imu_qx")
    imu_qy = LaunchConfiguration("imu_qy")
    imu_qz = LaunchConfiguration("imu_qz")
    imu_qw = LaunchConfiguration("imu_qw")
    imu_frame_id = LaunchConfiguration("imu_frame_id")

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
                "frame_id": lidar_frame_id,
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

    lidar_tf = Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            lidar_x,
            lidar_y,
            lidar_z,
            lidar_qx,
            lidar_qy,
            lidar_qz,
            lidar_qw,
            "base_link",
            lidar_frame_id,
        ],
        output="screen",
    )

    imu_node = Node(
        package="inertial_sense_ros2",
        executable="inertial_sense_ros2_node",
        name="inertial_sense",
        output="screen",
    )

    imu_tf = Node(
        name="imu_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            imu_x,
            imu_y,
            imu_z,
            imu_qx,
            imu_qy,
            imu_qz,
            imu_qw,
            "base_link",
            imu_frame_id,
        ],
        output="screen",
    )

    default_imu_qx = -0.7860113382339478
    default_imu_qy = 0.6103793382644653
    default_imu_qz = 0.0
    default_imu_qw = _default_imu_qw(default_imu_qx, default_imu_qy, default_imu_qz)

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
                "lidar_frame_id",
                default_value="velodyne",
                description="TF frame for LiDAR data",
            ),
            DeclareLaunchArgument(
                "lidar_x",
                default_value=str(1.524),
                description="LiDAR X offset (meters) in base_link frame",
            ),
            DeclareLaunchArgument(
                "lidar_y",
                default_value=str(0.0254),
                description="LiDAR Y offset (meters) in base_link frame",
            ),
            DeclareLaunchArgument(
                "lidar_z",
                default_value=str(1.778),
                description="LiDAR Z offset (meters) in base_link frame",
            ),
            DeclareLaunchArgument(
                "lidar_qx",
                default_value="0.0",
                description="LiDAR quaternion x (base_link -> lidar_frame_id)",
            ),
            DeclareLaunchArgument(
                "lidar_qy",
                default_value="0.0",
                description="LiDAR quaternion y (base_link -> lidar_frame_id)",
            ),
            DeclareLaunchArgument(
                "lidar_qz",
                default_value="0.0",
                description="LiDAR quaternion z (base_link -> lidar_frame_id)",
            ),
            DeclareLaunchArgument(
                "lidar_qw",
                default_value="1.0",
                description="LiDAR quaternion w (base_link -> lidar_frame_id)",
            ),
            DeclareLaunchArgument(
                "imu_frame_id",
                default_value="body",
                description="TF frame for IMU (must match /imu header.frame_id; inertial_sense_ros2 commonly uses 'body')",
            ),
            DeclareLaunchArgument(
                "imu_x",
                default_value="0.0",
                description="IMU X offset (meters) in base_link frame",
            ),
            DeclareLaunchArgument(
                "imu_y",
                default_value="0.0",
                description="IMU Y offset (meters) in base_link frame",
            ),
            DeclareLaunchArgument(
                "imu_z",
                default_value=str(0.9144),
                description="IMU Z offset (meters) in base_link frame (36 in = 0.9144 m)",
            ),
            DeclareLaunchArgument(
                "imu_qx",
                default_value=str(default_imu_qx),
                description="IMU quaternion x (base_link -> imu_frame_id)",
            ),
            DeclareLaunchArgument(
                "imu_qy",
                default_value=str(default_imu_qy),
                description="IMU quaternion y (base_link -> imu_frame_id)",
            ),
            DeclareLaunchArgument(
                "imu_qz",
                default_value=str(default_imu_qz),
                description="IMU quaternion z (base_link -> imu_frame_id)",
            ),
            DeclareLaunchArgument(
                "imu_qw",
                default_value=str(default_imu_qw),
                description="IMU quaternion w (base_link -> imu_frame_id)",
            ),
            velodyne_driver_node,
            velodyne_transform_launch,
            imu_node,
            lidar_tf,
            imu_tf,
        ]
    )
