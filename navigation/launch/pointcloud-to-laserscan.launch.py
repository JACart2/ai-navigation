from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
            remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
            parameters=[{'cloud_frame_id': 'cloud', 'cloud_extent': 2.0, 'cloud_size': 500}],
            name='cloud_publisher'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'cloud'
            ]
        ),
        # Lidar converter
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/cloud_in_lidar'),
                        ('scan', '/scanner/lidar_scan')],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': -5.0,  # More lenient height range
                'max_height': 5.0,
                'range_min': 0.1,   # Allow closer ranges
                'range_max': 50.0,  # Allow longer ranges
            }],
            name='pointcloud_to_laserscan_lidar'
        ),
        # Radar converter
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/cloud_in_radar'),
                        ('scan', '/scanner/radar_scan')],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': -5.0,  # More lenient height range for radar
                'max_height': 5.0,
                'range_min': 0.1,   # Allow closer ranges
                'range_max': 50.0,  # Allow longer ranges
            }],
            name='pointcloud_to_laserscan_radar'
        )
    ])
