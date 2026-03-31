""" Launch the motor control system.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


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
                default_value=os.path.join(
                    get_package_share_directory("navigation"), "maps", "main_shift3.gml"
                ),
            ),
            DeclareLaunchArgument(
                "enable_global_tester",
                default_value="false",
            ),
            Node(
                package="navigation",
                executable="global_planner",
                output="screen",
                parameters=[
                    {
                        "graph_file": LaunchConfiguration("graph_file"),
                    }
                ],
            ),
            Node(
                package="navigation",
                executable="display_global_path",
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
                condition=IfCondition(LaunchConfiguration("enable_global_tester")),
            ),
            Node(
                package="navigation",
                executable="visualize_graph",
                parameters=[
                    {
                        "graph_file": LaunchConfiguration("graph_file"),
                    }
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
