""" Launch the motor control system.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("navigation"),
                        "rviz/visualize_path.rviz",
                    ),
                ],
            ),
            DeclareLaunchArgument(
                "graph_file",
                default_value="/home/student/dev_ws/src/ai-navigation/navigation/navigation/drive_build.gml",
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
            Node(
                package="navigation",
                executable="motor_simulator",
            ),
            Node(
                package="navigation",
                executable="global_tester",
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
