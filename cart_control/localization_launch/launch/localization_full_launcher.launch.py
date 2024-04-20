from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    # Include the zed_camera launch file directly
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("zed_wrapper"),
                "/launch/zed_camera.launch.py",
            ]
        ),
        launch_arguments={"camera_model": "zed2"}.items(),
    )

    # Execute the RViz2 command with the specified configuration file
    rviz2_command = ExecuteProcess(
        cmd=["rviz2", "-d", "src/lidar_localization_ros2/rviz/localization.rviz"],
        shell=True,
    )

    # static_transform_publisher = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "run",
    #         "tf2_ros",
    #         "static_transform_publisher",
    #         "0.6",
    #         "0",
    #         "1",
    #         "0.0",
    #         "0.0",
    #         "0.0",
    #         "0.0",  # x, y, z, qx, qy, qz, qw
    #         "base_link",
    #         "odom",  # parent_frame_id, child_frame_id
    #     ],
    #     output="screen",
    # )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            velodyne_driver_node,
            velodyne_transform_launch,
            lidar_localization_launch,
            rviz2_command,
            zed_camera_launch,
            # static_transform_publisher,
            navigation_launch,
        ]
    )
