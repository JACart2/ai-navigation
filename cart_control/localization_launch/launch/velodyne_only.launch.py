import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


VALID_CARTS = ("james", "madison")


def _as_str(value):
    return str(value)


def _as_float(value):
    return float(value)


def _as_int(value):
    return int(value)


def _as_bool(value):
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("1", "true", "yes", "on"):
            return True
        if normalized in ("0", "false", "no", "off"):
            return False

    if isinstance(value, int) and value in (0, 1):
        return bool(value)

    raise ValueError(f"Expected a boolean value, got {value!r}.")


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


def _first_non_empty(*values):
    for value in values:
        if value not in ("", None):
            return value
    return None


def _make_velodyne_nodes(context, *args, **kwargs):
    cart = _selected_cart(context)
    cart_config_path = _cart_config_path(
        cart,
        LaunchConfiguration("cart_config_path").perform(context).strip(),
    )

    with open(cart_config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    velodyne = cfg.get("velodyne", {})

    model = _as_str(
        _first_non_empty(
            LaunchConfiguration("model").perform(context).strip(),
            velodyne.get("model"),
            "VLP16",
        )
    )
    rpm = _as_float(
        _first_non_empty(
            LaunchConfiguration("rpm").perform(context).strip(),
            velodyne.get("rpm"),
            600.0,
        )
    )
    device_ip = _as_str(
        _first_non_empty(
            LaunchConfiguration("device_ip").perform(context).strip(),
            velodyne.get("device_ip"),
            "192.168.1.201",
        )
    )
    port = _as_int(
        _first_non_empty(
            LaunchConfiguration("port").perform(context).strip(),
            velodyne.get("port"),
            2368,
        )
    )
    frame_id = _as_str(
        _first_non_empty(
            LaunchConfiguration("frame_id").perform(context).strip(),
            velodyne.get("frame_id"),
            "velodyne",
        )
    )

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

    return [
        LogInfo(
            msg=(
                f"Starting Velodyne for cart:={cart} using {cart_config_path} "
                f"(device_ip={device_ip}, port={port}, frame_id={frame_id})."
            )
        ),
        velodyne_driver_node,
        velodyne_transform_launch,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cart",
                default_value="james",
                description="Cart used to select cart-specific Velodyne settings.",
            ),
            DeclareLaunchArgument(
                "cart_name",
                default_value="",
                description="Legacy alias for cart.",
            ),
            DeclareLaunchArgument(
                "cart_config_path",
                default_value="",
                description="Optional explicit cart YAML path. Overrides cart.",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
                description=(
                    "Optional Velodyne model override. Empty means use "
                    "the selected cart config."
                ),
            ),
            DeclareLaunchArgument(
                "rpm",
                default_value="",
                description=(
                    "Optional LiDAR spin rate override. Empty means use "
                    "the selected cart config."
                ),
            ),
            DeclareLaunchArgument(
                "device_ip",
                default_value="",
                description=(
                    "Optional Velodyne sensor IP override. Empty means use "
                    "the selected cart config."
                ),
            ),
            DeclareLaunchArgument(
                "port",
                default_value="",
                description=(
                    "Optional Velodyne UDP data port override. Empty means "
                    "use the selected cart config."
                ),
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description=(
                    "Optional TF frame override. Empty means use the selected "
                    "cart config."
                ),
            ),
            OpaqueFunction(function=_make_velodyne_nodes),
        ]
    )
