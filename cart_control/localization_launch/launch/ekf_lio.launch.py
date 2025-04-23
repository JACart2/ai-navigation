from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_lio_node',
            output='screen',
            parameters=['/dev_ws/src/ai-navigation/cart_control/localization_launch/config/ekf_lio.yaml']
        )
    ])