from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/cloud_in'),
                        ('scan', '/scanner/scan')],
            parameters=[{
                'min_height': -1.0,
                'max_height': 0.0,
                'range_min': 0.8,
            }],
            name='pointcloud_to_laserscan'
        )
    ])
