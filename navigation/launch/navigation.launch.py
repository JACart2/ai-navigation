""" Launch the motor control system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
                "radar_min_height",
                default_value="-0.10",
            ),
            DeclareLaunchArgument(
                "radar_max_height",
                default_value="0.75",
            ),
            DeclareLaunchArgument(
                "radar_min_cluster_size",
                default_value="2",
            ),
            DeclareLaunchArgument(
                "radar_persistence_frames",
                default_value="3",
            ),
            DeclareLaunchArgument(
                "radar_persistence_match_radius",
                default_value="0.75",
            ),
            DeclareLaunchArgument(
                "radar_track_timeout_sec",
                default_value="0.40",
            ),
            DeclareLaunchArgument("radar_filter_x_min", default_value="0.25"),
            DeclareLaunchArgument("radar_filter_x_max", default_value="3.58"),
            DeclareLaunchArgument("radar_filter_y_min", default_value="-2.10"),
            DeclareLaunchArgument("radar_filter_y_max", default_value="5.00"),
            DeclareLaunchArgument("radar_filter_z_min", default_value="-0.65"),
            DeclareLaunchArgument("radar_filter_z_max", default_value="1.11"),
            DeclareLaunchArgument(
                "radar_filter_gui",
                default_value="true",
                description="Open the Tk slider window for the radar xyz filter.",
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
                ),
                launch_arguments={
                    "radar_min_cluster_size": LaunchConfiguration("radar_min_cluster_size"),
                    "radar_persistence_frames": LaunchConfiguration("radar_persistence_frames"),
                    "radar_persistence_match_radius": LaunchConfiguration("radar_persistence_match_radius"),
                    "radar_track_timeout_sec": LaunchConfiguration("radar_track_timeout_sec"),
                    "radar_filter_x_min": LaunchConfiguration("radar_filter_x_min"),
                    "radar_filter_x_max": LaunchConfiguration("radar_filter_x_max"),
                    "radar_filter_y_min": LaunchConfiguration("radar_filter_y_min"),
                    "radar_filter_y_max": LaunchConfiguration("radar_filter_y_max"),
                    "radar_filter_z_min": LaunchConfiguration("radar_filter_z_min"),
                    "radar_filter_z_max": LaunchConfiguration("radar_filter_z_max"),
                    "radar_filter_gui": LaunchConfiguration("radar_filter_gui"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("navigation"), "/launch/pointcloud-to-laserscan.launch.py"]
                ),
                launch_arguments={
                    "radar_min_height": LaunchConfiguration("radar_min_height"),
                    "radar_max_height": LaunchConfiguration("radar_max_height"),
                }.items(),
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
