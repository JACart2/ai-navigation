from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import launch_ros.actions
import launch_ros.events
 
 
def generate_launch_description():
    """
    Generate a launch description for the full localization stack.
    
    This includes:
    - Velodyne LiDAR driver and transform
    - LiDAR-based localization
    - One ZED camera (front only)
    - Static transform between base_link and front camera
    """
 
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
    # Specify the path to zed_camera.launch.py
    zed_camera_launch_path = "./src/ai-navigation/cart_control/localization_launch/launch/zed_camera.launch.py"
 
    # Include the lidar_localization launch file using the new path
    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_localization_launch_path])
    )
 
    # Launch front camera with only name and model parameters
    zed_front_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([zed_camera_launch_path]),
        launch_arguments={
            "camera_name": "zed_front",
            "camera_model": "zed2i"
        }.items(),
    )
 
    # Static transform for the front camera to base_link
    front_camera_tf = ExecuteProcess(
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
            "zed_front_camera_link",  # Child frame (front camera link)
        ],
        output="screen",
    )
    
    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            velodyne_driver_node,
            velodyne_transform_launch,
            lidar_localization_launch,
            zed_front_launch,
            front_camera_tf,
        ]
    )