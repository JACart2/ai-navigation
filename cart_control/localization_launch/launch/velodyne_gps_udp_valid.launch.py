from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    raw_topic = LaunchConfiguration("raw_topic")
    valid_topic = LaunchConfiguration("valid_topic")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "host",
                default_value="0.0.0.0",
                description="Host address to bind for Velodyne GPS UDP packets",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8308",
                description="UDP port for Velodyne GPS/position packets",
            ),
            DeclareLaunchArgument(
                "raw_topic",
                default_value="/velodyne_fix",
                description="Raw NavSatFix topic from Velodyne NMEA packets",
            ),
            DeclareLaunchArgument(
                "valid_topic",
                default_value="/velodyne_fix_valid",
                description="Filtered valid NavSatFix topic",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="gps",
                description="Frame id for published GPS fixes",
            ),
            Node(
                package="localization_launch",
                executable="velodyne_gps_udp_driver",
                name="velodyne_gps_udp_driver",
                output="screen",
                parameters=[
                    {
                        "host": ParameterValue(host, value_type=str),
                        "port": ParameterValue(port, value_type=int),
                        "output_topic": ParameterValue(raw_topic, value_type=str),
                        "frame_id": ParameterValue(frame_id, value_type=str),
                    }
                ],
            ),
            Node(
                package="localization_launch",
                executable="valid_fix_filter",
                name="velodyne_valid_fix_filter",
                output="screen",
                parameters=[
                    {
                        "input_topic": ParameterValue(raw_topic, value_type=str),
                        "output_topic": ParameterValue(valid_topic, value_type=str),
                    }
                ],
            ),
        ]
    )
