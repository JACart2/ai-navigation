from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="navigation",
            executable="zed_object_to_obstacle",
            output="screen",
        ),
        Node(
            package="navigation",
            executable="lidar_object_to_obstacle",
            output="screen",
        ),
        Node(
            package="navigation",
            executable="collision_detector",
            output="screen",
        ),
    ])
