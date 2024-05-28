from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import launch_ros.actions
import launch_ros.events

def generate_launch_description():

    # Launch the localization launcher
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("localization_launch"), "/launch/localization_full_launcher.launch.py"]
        )
    )

    # Launch the navigation launcher
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("navigation"), "/launch/navigation.launch.py"]
        )
    )

    # Launch the motor control launcher
    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("motor_control"), "/launch/motor.launch.py"]
        )
    )

    # Combine all the above components into a single launch description
    return LaunchDescription(
        [
            localization_launch,
            navigation_launch,
            motor_control_launch,
        ]
    )
