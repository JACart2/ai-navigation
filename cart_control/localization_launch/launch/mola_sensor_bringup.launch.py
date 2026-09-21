import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_str(value):
    return str(value)


VALID_CARTS = ("james", "madison")


def _selected_cart(context):
    cart = LaunchConfiguration("cart").perform(context).strip().lower()
    cart_name = LaunchConfiguration("cart_name").perform(context).strip().lower()
    selected = cart_name or cart or "james"

    if selected not in VALID_CARTS:
        raise RuntimeError(
            f"Invalid cart '{selected}'. Expected one of: {', '.join(VALID_CARTS)}."
        )

    return selected


def _load_cart_config(context, *args, **kwargs):
    """
    Cart-aware MOLA sensor bringup.

    Reads Velodyne network/settings from cart_config_path.

    If the YAML later contains a verified lidar_tf block, this launch file will
    use it. Otherwise it preserves the existing MOLA/default static transform:
      base_link -> velodyne = 0.5, 0.0, 1.75, 0, 0, 0
    """

    cart = _selected_cart(context)
    cart_config_path = LaunchConfiguration("cart_config_path").perform(context).strip()

    if not cart_config_path:
        cart_config_path = os.path.join(
            get_package_share_directory("cart_launch"),
            "config",
            f"cart_{cart}.yaml",
        )

    lidar_topic_name = LaunchConfiguration("lidar_topic_name").perform(context)
    lidar_topic_type = LaunchConfiguration("lidar_topic_type").perform(context)
    start_active = LaunchConfiguration("start_active").perform(context)

    default_lidar_x = LaunchConfiguration("lidar_x").perform(context)
    default_lidar_y = LaunchConfiguration("lidar_y").perform(context)
    default_lidar_z = LaunchConfiguration("lidar_z").perform(context)
    default_lidar_yaw = LaunchConfiguration("lidar_yaw").perform(context)
    default_lidar_pitch = LaunchConfiguration("lidar_pitch").perform(context)
    default_lidar_roll = LaunchConfiguration("lidar_roll").perform(context)
    default_base_frame = LaunchConfiguration("base_frame").perform(context)
    default_lidar_frame = LaunchConfiguration("lidar_frame").perform(context)

    with open(cart_config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    velodyne = cfg.get("velodyne", {})
    lidar_tf = cfg.get("lidar_tf", {})

    model = _as_str(velodyne.get("model", "VLP16"))
    rpm = _as_str(velodyne.get("rpm", 600.0))
    device_ip = _as_str(velodyne.get("device_ip", "192.168.1.201"))
    port = _as_str(velodyne.get("port", 2368))
    frame_id = _as_str(velodyne.get("frame_id", default_lidar_frame))

    parent_frame = _as_str(lidar_tf.get("parent_frame", default_base_frame))
    child_frame = _as_str(lidar_tf.get("child_frame", frame_id))
    x = _as_str(lidar_tf.get("x", default_lidar_x))
    y = _as_str(lidar_tf.get("y", default_lidar_y))
    z = _as_str(lidar_tf.get("z", default_lidar_z))
    yaw = _as_str(lidar_tf.get("yaw", default_lidar_yaw))
    pitch = _as_str(lidar_tf.get("pitch", default_lidar_pitch))
    roll = _as_str(lidar_tf.get("roll", default_lidar_roll))

    localization_launch_share = get_package_share_directory("localization_launch")

    velodyne_launch_path = os.path.join(
        localization_launch_share,
        "launch",
        "velodyne_only.launch.py",
    )

    mola_lidar_odometry_launch_path = os.path.join(
        localization_launch_share,
        "launch",
        "mola_lidar_odometry.launch.py",
    )

    base_link_to_velodyne_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_velodyne_tf",
        arguments=[
            x,
            y,
            z,
            yaw,
            pitch,
            roll,
            parent_frame,
            child_frame,
        ],
        output="screen",
    )

    velodyne_only_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_launch_path),
        launch_arguments={
            "model": model,
            "rpm": rpm,
            "device_ip": device_ip,
            "port": port,
            "frame_id": frame_id,
        }.items(),
    )

    mola_lidar_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mola_lidar_odometry_launch_path),
        launch_arguments={
            "lidar_topic_name": lidar_topic_name,
            "lidar_topic_type": lidar_topic_type,
            "start_active": start_active,
        }.items(),
    )

    return [
        base_link_to_velodyne_tf,
        velodyne_only_launch,
        mola_lidar_odometry_launch,
    ]


def generate_launch_description():
    """
    Safe MOLA bringup for Velodyne LiDAR + MOLA odometry.

    This launch file does not start:
    - old lidar_localization
    - pcl_pose_relay
    - cameras
    - navigation
    - motor control
    - mapping/simplemap saving
    """

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart",
                default_value="james",
                choices=["james", "madison"],
                description="Cart used to auto-select cart_james.yaml or cart_madison.yaml.",
            ),
            DeclareLaunchArgument(
                "cart_name",
                default_value="",
                description="Legacy alias for cart.",
            ),
            DeclareLaunchArgument(
                "cart_config_path",
                default_value="",
                description="Optional explicit path to cart-specific YAML config. Overrides cart_name when set.",
            ),
            DeclareLaunchArgument(
                "lidar_topic_name",
                default_value="/velodyne_points",
                description="Input LiDAR topic for MOLA odometry.",
            ),
            DeclareLaunchArgument(
                "lidar_topic_type",
                default_value="PointCloud2",
                description="Input LiDAR message type. Must be exactly PointCloud2.",
            ),
            DeclareLaunchArgument(
                "start_active",
                default_value="True",
                description="Start MOLA odometry active by default.",
            ),

            # Existing safe/default base_link -> velodyne transform.
            # Do not treat as final per-cart measurement.
            DeclareLaunchArgument("lidar_x", default_value="0.5"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="1.75"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("lidar_frame", default_value="velodyne"),

            OpaqueFunction(function=_load_cart_config),
        ]
    )
