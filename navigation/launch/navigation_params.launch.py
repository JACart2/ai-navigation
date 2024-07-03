import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def generate_launch_description():
    # Define the parameters
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

    # # Node configuration
    # gps_node = Node(
    #     package='your_package_name',  # Replace with your package name
    #     executable='your_node_executable',  # Replace with your node executable name
    #     name='gps_node',
    #     output='screen',
    #     parameters=[gps_util_params]
    # )

    return LaunchDescription([
        declare_graph_file_cmd,
        # gps_node
    ])


if __name__ == '__main__':
    generate_launch_description()
