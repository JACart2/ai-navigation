import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
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


def _cart_config_path(cart, cart_config_path):
    if cart_config_path:
        return cart_config_path
    return os.path.join(
        get_package_share_directory("cart_launch"),
        "config",
        f"cart_{cart}.yaml",
    )


def _make_lidar_static_tf(context, *args, **kwargs):
    cart = _selected_cart(context)
    cart_config_path = (
        LaunchConfiguration("cart_config_path").perform(context).strip()
    )
    base_frame = LaunchConfiguration("base_frame").perform(context)
    lidar_frame = LaunchConfiguration("lidar_frame").perform(context)
    default_lidar_x = LaunchConfiguration("lidar_x").perform(context)
    default_lidar_y = LaunchConfiguration("lidar_y").perform(context)
    default_lidar_z = LaunchConfiguration("lidar_z").perform(context)
    default_lidar_yaw = LaunchConfiguration("lidar_yaw").perform(context)
    default_lidar_pitch = LaunchConfiguration("lidar_pitch").perform(context)
    default_lidar_roll = LaunchConfiguration("lidar_roll").perform(context)

    cfg = {}
    resolved_config_path = _cart_config_path(cart, cart_config_path)
    if resolved_config_path and os.path.exists(resolved_config_path):
        with open(resolved_config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}

    lidar_tf = cfg.get("lidar_tf", {})
    info_actions = []
    if not lidar_tf:
        info_actions.append(
            LogInfo(
                msg=(
                    "No lidar_tf configured for cart:="
                    f"{cart}; using shared MOLA/default base_link->velodyne "
                    "static TF."
                )
            )
        )
        lidar_tf = {
            "parent_frame": base_frame,
            "child_frame": lidar_frame,
            "x": default_lidar_x,
            "y": default_lidar_y,
            "z": default_lidar_z,
            "roll": default_lidar_roll,
            "pitch": default_lidar_pitch,
            "yaw": default_lidar_yaw,
        }

    parent_frame = _as_str(lidar_tf.get("parent_frame", base_frame))
    child_frame = _as_str(lidar_tf.get("child_frame", lidar_frame))
    x = _as_str(lidar_tf.get("x", default_lidar_x))
    y = _as_str(lidar_tf.get("y", default_lidar_y))
    z = _as_str(lidar_tf.get("z", default_lidar_z))
    roll = _as_str(lidar_tf.get("roll", default_lidar_roll))
    pitch = _as_str(lidar_tf.get("pitch", default_lidar_pitch))
    yaw = _as_str(lidar_tf.get("yaw", default_lidar_yaw))

    return info_actions + [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_velodyne_tf",
            arguments=[x, y, z, yaw, pitch, roll, parent_frame, child_frame],
            output="screen",
        )
    ]


def _make_rviz(context, *args, **kwargs):
    use_rviz = LaunchConfiguration("use_rviz").perform(context).strip().lower()
    if use_rviz not in ("1", "true", "yes", "on"):
        return []

    rviz_config = (
        "/opt/ros/jazzy/share/mola_lidar_odometry/rviz2/lidar-odometry.rviz"
    )
    if not os.path.exists(rviz_config):
        return [
            LogInfo(
                msg=(
                    f"MOLA RViz config not found: {rviz_config}; "
                    "skipping RViz."
                )
            )
        ]

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    ]


def generate_launch_description():
    map_file = LaunchConfiguration("map_file")
    lidar_topic = LaunchConfiguration("lidar_topic")
    base_frame = LaunchConfiguration("base_frame")

    mola_launch_path = os.path.join(
        get_package_share_directory("mola_lidar_odometry"),
        "ros2-launchs",
        "ros2-lidar-odometry.launch.py",
    )

    mola_lidar_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mola_launch_path),
        launch_arguments={
            "start_active": "True",
            "publish_localization_following_rep105": "False",
            "start_mapping_enabled": "False",
            "lidar_topic_name": lidar_topic,
            "mola_tf_base_link": base_frame,
            "lidar_scan_validity_minimum_point_count": "20",
            "mola_initial_map_mm_file": map_file,
        }.items(),
    )

    odom_tf_node = Node(
        package="localization_launch",
        executable="mola_odom_to_tf",
        name="mola_odom_to_tf",
        output="screen",
        parameters=[
            {
                "odom_topic": "/lidar_odometry/pose",
                "parent_frame": "map",
                "child_frame": base_frame,
                "publish_rate_hz": 20.0,
                "use_msg_stamp": False,
            }
        ],
        condition=IfCondition(LaunchConfiguration("publish_odom_tf")),
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument("lidar_x", default_value="0.5"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="1.75"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument(
                "cart",
                default_value="james",
                description=(
                    "Cart name used to auto-select cart-specific config."
                ),
            ),
            DeclareLaunchArgument(
                "cart_name",
                default_value="",
                description="Legacy alias for cart.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description=(
                    "Start RViz with MOLA's LiDAR odometry config "
                    "when available."
                ),
            ),
            DeclareLaunchArgument(
                "publish_odom_tf",
                default_value="true",
                description="Publish dynamic TF from MOLA odometry.",
            ),
            DeclareLaunchArgument(
                "cart_config_path",
                default_value="",
                description=(
                    "Optional explicit path to cart-specific YAML config."
                ),
            ),
            SetEnvironmentVariable(
                "MOLA_MAX_TIME_TO_USE_VELOCITY_MODEL",
                "2.0",
            ),
            OpaqueFunction(function=_make_lidar_static_tf),
            mola_lidar_odometry_launch,
            odom_tf_node,
            OpaqueFunction(function=_make_rviz),
        ]
    )
