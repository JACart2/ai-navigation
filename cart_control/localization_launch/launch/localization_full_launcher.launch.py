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

    # Include the lidar_localization launch file directly
    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("lidar_localization_ros2"),
                "/launch/lidar_localization.launch.py",
            ]
        )
    )

    # Include the navigation launch file directly
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("navigation"),
                "/launch/navigation.launch.py",
            ]
        )
    )

    # Include the navigation launch file directly
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("motor_control"),
                "/launch/motor.launch.py",
            ]
        )
    )

    # # Include the zed_camera launch file directly
    # zed_camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             FindPackageShare("zed_wrapper"),
    #             "/launch/zed_camera.launch.py",
    #         ]
    #     ),
    #     launch_arguments={"camera_model": "zed2"}.items(),
    # )

    # Include the zed_multi_camera launch file instead of individual zed_camera launches
    zed_multi_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("zed_multi_camera"),
                "/launch/zed_multi_camera.launch.py",
            ]
        ),
        # Pass launch arguments for cameras, models, serials, and TF configuration
        launch_arguments={
            "cam_names": "[zed_front, zed_rear]",  # Names of the cameras
            "cam_models": "[zed2i, zed2i]",  # Models of the cameras
            "cam_serials": "[37963597, 31061594]",  # Serial numbers of the cameras
            "disable_tf": "False",  # Enable TF broadcasting
        }.items(),
    )

    # # Static transform for the reference link (zed_multi_link) to base_link
    # multi_link_tf = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "run",
    #         "tf2_ros",
    #         "static_transform_publisher",
    #         "0.0",  # X position (adjust as needed)
    #         "0.0",  # Y position (adjust as needed)
    #         "1.6",  # Z position (height of cameras above ground)
    #         "0.0",  # Roll (rotation around X-axis)
    #         "0.0",  # Pitch (rotation around Y-axis)
    #         "0.0",  # Yaw (rotation around Z-axis)
    #         "base_link",  # Parent frame (golf cart base)
    #         "zed_front_camera_link",  # Child frame (reference link for cameras)
    #     ],
    #     output="screen",
    # )

    # Execute the RViz2 command with the specified configuration file
    rviz2_command = ExecuteProcess(
        cmd=["rviz2", "-d", "src/lidar_localization_ros2/rviz/localization.rviz"],
        shell=True,
    )

    static_transform_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0.8",
            "0",
            "1",
            "0",
            "0",
            "0",
            "1",  # x, y, z, qx, qy, qz, qw
            "base_link",
            "odom",  # parent_frame_id, child_frame_id
        ],
        output="screen",
    )

    lidar_tf = launch_ros.actions.Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.6", "0", "1", "0", "0", "0", "1", "base_link", "velodyne"],
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            velodyne_driver_node,
            velodyne_transform_launch,
            lidar_localization_launch,
            rviz2_command,
            # zed_camera_launch,
            zed_multi_camera_launch,
            # multi_link_tf,
            # static_transform_publisher,
            # navigation_launch,
            # motor_launch,
        ]
    )
