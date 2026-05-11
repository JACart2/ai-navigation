from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)

def generate_launch_description():
    return LaunchDescription([
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
            executable="zed_object_to_obstacle",
            output="screen",
        ),
        Node(
            package="navigation",
            executable="radar_xyz_filter",
            name="radar_xyz_filter",
            output="screen",
            parameters=[{
                "x_min": _f("radar_filter_x_min"),
                "x_max": _f("radar_filter_x_max"),
                "y_min": _f("radar_filter_y_min"),
                "y_max": _f("radar_filter_y_max"),
                "z_min": _f("radar_filter_z_min"),
                "z_max": _f("radar_filter_z_max"),
            }],
        ),
        Node(
            package="navigation",
            executable="radar_filter_sliders",
            name="radar_filter_sliders",
            output="screen",
            condition=IfCondition(LaunchConfiguration("radar_filter_gui")),
        ),
        Node(
            package="navigation",
            executable="radar_pcl_to_obstacles",
            name="radar_pcl_to_obstacles",
            output="screen",
        ),
        Node(
            package="navigation",
            executable="lidar_object_to_obstacle",
            output="screen",
            parameters=[
                {
                    "radar_min_cluster_size": LaunchConfiguration("radar_min_cluster_size"),
                    "radar_persistence_frames": LaunchConfiguration("radar_persistence_frames"),
                    "radar_persistence_match_radius": LaunchConfiguration("radar_persistence_match_radius"),
                    "radar_track_timeout_sec": LaunchConfiguration("radar_track_timeout_sec"),
                }
            ],
            remappings=[
                ("/ti_mmwave/radar_scan_pcl", "/ti_mmwave/radar_scan_pcl_filtered"),
            ],
        ),
        Node(
            package="navigation",
            executable="collision_detector",
            output="screen",
        ),
    ])
