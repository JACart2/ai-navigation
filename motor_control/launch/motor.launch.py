"""Launch the Zeta rescue system 

You can make any changes you want to this launch file, but it must
accept the time_limit and use_sim_time command line arguments.

"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("baudrate", default_value="57600"),
            DeclareLaunchArgument("arduino_port", default_value="dev/ttyACM0"),
            Node(
                package="motor_control",
                executable="motor_endpoint",
                output="screen",
                parameters=[
                    {
                        "baudrate": LaunchConfiguration("baudrate"),
                        "arduino_port": LaunchConfiguration("arduino_port"),
                    }
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

if __name__ == "__main__":
    generate_launch_description()
