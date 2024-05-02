""" Launch the motor control system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "graph_file",
                default_value="/home/student/dev_ws/src/ai-navigation/navigation/maps/main.gml",
            ),
            Node(
                package="navigation",
                executable="global_planner",
                output="screen",
            ),
            Node(
                package="navigation",
                executable="local_planner",
                output="screen",
            ),
            # Run the speed_node
            Node(
                package="navigation",
                executable="speed_node",
                output="screen",
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
