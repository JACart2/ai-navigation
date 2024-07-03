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
                "graph_file",
                default_value="./src/ai-navigation/navigation/maps/main_shift3.gml",
            ),
            DeclareLaunchArgument(
                "anchor_gps",
                default_value="38.433939, -78.862157",
            ),
            DeclareLaunchArgument(
                "anchor_theta",
                default_value="100",
            ),
            DeclareLaunchArgument(
                "anchor_local",
                default_value="0, 0",
            ),
            DeclareLaunchArgument(
                "test_loc_gps",
                default_value="38.433170, -78.860981",
            ),
            DeclareLaunchArgument(
                "test_loc_local",
                default_value="67.6, 115",
            ),
            Node(
                package='navigation',
                executable='global_planner',
                output='screen',
                parameters=[
                    {
                        'anchor_local': LaunchConfiguration('anchor_local'),
                        'test_loc_gps': LaunchConfiguration('test_loc_gps'),
                        'test_loc_local': LaunchConfiguration('test_loc_local'),
                        'anchor_gps': LaunchConfiguration('anchor_gps'),
                        'anchor_theta': LaunchConfiguration('anchor_theta'),
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
                executable="motor_simulator",
                output="screen",
            ),
            Node(
                package="navigation",
                executable="global_tester",
                output="screen",
            ),
            Node(
                package="navigation",
                executable="visualize_graph",
                output="screen",
                parameters=[
                    {
                        "graph_file": LaunchConfiguration("graph_file"),

                    }
                ],
            ),
            gps_util_params = {
            'anchor_gps': [38.433939, -78.862157],
            'anchor_theta': 100,
            'anchor_local': [0, 0],
            'test_loc_gps': [38.433170, -78.860981],
            'test_loc_local': [67.6, 115],
            'max_speed': 5,
            'max_ndt_health': 4,
            'min_obstacle_dist': 0.5,
            'min_obstacle_time': 0.5,
            'safe_obstacle_dist': 6,
            'safe_obstacle_time': 2,
            'vehicle_width': 1.1938,
            'vehicle_length': 2.4003,
            'wheel_base': 2.4003,
            'front_axle_track': 0.9017,
            'rear_axle_track': 0.9652,
            'tire_width': 0.2159
        }
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
