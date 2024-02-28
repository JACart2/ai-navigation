""" Launch the motor control system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("graph_file"),

            Node(
                package="navigation",
                executable="global_planner",
                output="screen",
             
            ),
            Node(
            package="navigation",
            executable="local_planner",
            output="screen",
           
            
        )
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

if __name__ == "__main__":
    generate_launch_description()
