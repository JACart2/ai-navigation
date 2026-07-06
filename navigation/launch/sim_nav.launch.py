""" Launch the motor control system.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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
                "graph_dir",
                default_value=os.path.join(
                    get_package_share_directory("navigation"), "maps"
                ),
            ),
            DeclareLaunchArgument(
                "graph_file",
                default_value="main_shift3.gml",
            ),
            DeclareLaunchArgument(
                "graph_coordinate_format",
                default_value="ros",
            ),
            DeclareLaunchArgument(
                "calibration_config_dir",
                default_value="/maps",
            ),
            DeclareLaunchArgument(
                "calibration_config_file",
                default_value="SpeedBoiMap.yaml",
            ),
            Node(
                package="navigation",
                executable="global_planner",
                output="screen",
                parameters=[
                    {
                        "graph_dir": LaunchConfiguration("graph_dir"),
                        "graph_file": LaunchConfiguration("graph_file"),
                        "graph_coordinate_format": LaunchConfiguration("graph_coordinate_format"),
                        "calibration_config_dir": LaunchConfiguration("calibration_config_dir"),
                        "calibration_config_file": LaunchConfiguration("calibration_config_file"),
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
            ),
            Node(
                package="navigation",
                executable="visualize_graph",
                parameters=[
                    {
                        "graph_dir": LaunchConfiguration("graph_dir"),
                        "graph_file": LaunchConfiguration("graph_file"),
                        "graph_coordinate_format": LaunchConfiguration("graph_coordinate_format"),
                        "calibration_config_dir": LaunchConfiguration("calibration_config_dir"),
                        "calibration_config_file": LaunchConfiguration("calibration_config_file"),
                    }
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
