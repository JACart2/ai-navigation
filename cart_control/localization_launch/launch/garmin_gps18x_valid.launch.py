from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyACM0",
                description="Serial port for Garmin GPS 18x",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="4800",
                description="Serial baud rate for the GPS",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/fix",
                description="Raw GPS NavSatFix topic",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/fix_valid",
                description="Filtered valid GPS NavSatFix topic",
            ),
            Node(
                package="localization_launch",
                executable="garmin_gps18x_driver",
                name="garmin_gps18x_driver",
                output="screen",
                parameters=[
                    {
                        "port": ParameterValue(port, value_type=str),
                        "baud": ParameterValue(baud, value_type=int),
                    }
                ],
            ),
            Node(
                package="localization_launch",
                executable="valid_fix_filter",
                name="valid_fix_filter",
                output="screen",
                parameters=[
                    {
                        "input_topic": ParameterValue(input_topic, value_type=str),
                        "output_topic": ParameterValue(output_topic, value_type=str),
                    }
                ],
            ),
        ]
    )
