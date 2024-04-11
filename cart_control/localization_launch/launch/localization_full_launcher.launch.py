from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch the velodyne_driver node for VLP16
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        parameters=[{'model': 'VLP16'}],
    )

    # Launch the velodyne_transform_node for VLP16 from velodyne_pointcloud package
    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        parameters=[{
            'model': 'VLP16',
            'calibration': '/home/sensors/dev_ws/src/velodyne/velodyne_pointcloud/params/VLP16db.yaml'
        }],
    )

    # Include the lidar_localization launch file directly
    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('lidar_localization_ros2'),
            '/launch/lidar_localization.launch.py'
        ])
    )

    # Execute the RViz2 command with the specified configuration file
    rviz2_command = ExecuteProcess(
        cmd=['rviz2', '-d', 'src/lidar_localization_ros2/rviz/localization.rviz'],
        shell=True
    )

    # Combine all the above components into a single launch description
    return LaunchDescription([
        velodyne_driver_node,
        velodyne_transform_node,
        lidar_localization_launch,
        rviz2_command,
    ])
