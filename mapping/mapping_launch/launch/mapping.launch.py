from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
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

    lidar_tf = launch_ros.actions.Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["1", "0", "1.9", "0", "0", "0", "1", "base_link", "velodyne"],
    )

    mola = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("mola_lidar_odometry"),
                "/ros2-launchs/ros2-lidar-odometry.launch.py" # Yes, they spelled it "launchs"
            ]
        ),
        launch_arguments={
            "lidar_topic_name": "velodyne_points"
        }.items()
    )

    # Execute the RViz2 command with the specified configuration file
    rviz2_command = ExecuteProcess(
        cmd=[
            "rviz2",
            "-d",
            "src/ai-navigation/cart_control/cart_launch/rviz/localization.rviz",
        ],
        shell=True,
    )

    # GPS
    # Launch to test it:
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("mrpt_sensor_gnss_nmea"),
                "/launch/mrpt_sensor_gnss_nmea.launch.py" # Yes, they spelled it "launchs"
            ]
        ),
        launch_arguments={
            "serial_port": "/dev/ttyACM0",
            "publish_topic" : "/gps"
        }.items()
    )
    # gps_launch = LaunchDescription(
    #     Node(
    #         package='nmea_navsat_driver',
    #         executable='nmea_navsat_driver_node',
    #         name='nmea_navsat_driver',
    #         output='screen',
    #         parameters=[{
    #             'serial_port': '/dev/ttyACM0',
    #             'serial_baud': 4800
    #         }],
    #         remappings=[('/fix', '/gps')]
    #     )
    # )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            velodyne_driver_node,
            velodyne_transform_launch,
            lidar_tf,
            mola,
            rviz2_command,
            gps_launch
        ]
    )
