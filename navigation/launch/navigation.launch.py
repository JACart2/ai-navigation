""" Launch the motor control system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory 
import os


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "graph_file",
                default_value=os.path.join(
                    get_package_share_directory("navigation"), "maps", "main_shift3.gml"
                ),
            ),
            DeclareLaunchArgument(
                "enable_aad",
                default_value="true",
                description="Enable collision avoidance anomaly logging node"
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
                executable="local_planner",
                output="screen",
            ),
            Node(
                package="navigation",
                executable="display_global_path",
                output="screen",
            ),
            # Run the speed_node
            Node(
                package="navigation",
                executable="speed_node",
                output="screen",
            ),
            Node(
                package="navigation",
                executable="visualize_graph",
                output="screen",
            ),
            Node(
                package="navigation",
                executable="pose_bridge",
                output="screen",
            ),
            # call other launchfiles from this package:
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("navigation"), "/launch/obstacle_conversion.launch.py"]
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("navigation"), "/launch/pointcloud-to-laserscan.launch.py"]
                )
            ),
            Node(
                package="navigation",
                executable="collision_avoidance_aad_log",
                name="collision_avoidance_anomaly_log",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_aad")),
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
