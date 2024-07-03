from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import launch_ros.actions
import launch_ros.events


def generate_launch_description():

    # Launch the velodyne_driver node for VLP16
    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        parameters=[{"model": "VLP16"}],
    )

    # Include the velodyne_transform_node-VLP16-launch.py directly
    velodyne_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("velodyne_pointcloud"),
                "/launch/velodyne_transform_node-VLP16-launch.py",
            ]
        )
    )

    # Specify the new path to lidar_localization.launch.py
    lidar_localization_launch_path = "./src/ai-navigation/cart_control/localization_launch/launch/lidar_localization.launch.py"
    # Specify the new path to zed_multi_camera.launch.py
    zed_multi_camera_launch_path = "./src/ai-navigation/cart_control/localization_launch/launch/zed_multi_camera.launch.py"

    # Include the lidar_localization launch file using the new path
    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_localization_launch_path])
    )

    # Include the zed_multi_camera launch file instead of individual zed_camera launches
    zed_multi_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([zed_multi_camera_launch_path]),
        # Pass launch arguments for cameras, models, serials, and TF configuration
        launch_arguments={
            "cam_names": "[zed_front, zed_rear]",  # Names of the cameras
            "cam_models": "[zed2i, zed2i]",  # Models of the cameras
            "cam_serials": "[37963597, 31061594]",  # Serial numbers of the cameras
            "disable_tf": "False",  # Enable TF broadcasting
        }.items(),
    )

    # Static transform for the reference link (zed_multi_link) to base_link
    multi_link_tf = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "1.0",  # X position (adjust as needed)
            "0.0",  # Y position (adjust as needed)
            "1.6",  # Z position (height of cameras above ground)
            "0.0",  # Roll (rotation around X-axis)
            "0.0",  # Pitch (rotation around Y-axis)
            "0.0",  # Yaw (rotation around Z-axis)
            "base_link",  # Parent frame (golf cart base)
            "zed_front_camera_link",  # Child frame (reference link for cameras)
        ],
        output="screen",
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            velodyne_driver_node,
            velodyne_transform_launch,
            lidar_localization_launch,
            zed_multi_camera_launch,
            multi_link_tf,
        ]
    )
