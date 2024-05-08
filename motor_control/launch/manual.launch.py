""" Launch the motor control system with manual control through teleop.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("baudrate", default_value="57600"),
            DeclareLaunchArgument("arduino_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("manual_control", default_value="True"),
            Node(
                package="motor_control",
                executable="motor_endpoint",
                output="screen",
                parameters=[
                    {
                        "baudrate": LaunchConfiguration("baudrate"),
                        "arduino_port": LaunchConfiguration("arduino_port"),
                        "manual_control": LaunchConfiguration("manual_control"),
                    }
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
