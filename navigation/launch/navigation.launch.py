from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "graph_file",
                default_value="./src/ai-navigation/navigation/maps/main_shift3.gml",
            ),
            Node(
                package="navigation",
                executable="global_planner",
                output="screen",
                parameters=[
                    {
                        "graph_file": LaunchConfiguration("graph_file"),
                        **gps_util_params
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
                executable="speed_node",
                output="screen",
            ),
            Node(
                package="navigation",
                executable="visualize_graph",
                output="screen",
            ),
        ]
    )

if __name__ == "__main__":
    generate_launch_description()
