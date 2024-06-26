import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to the rosbag
    rosbag_path = "rosbag2_2024_06_05-10_56_12/"

    # Path to the rviz config file
    rviz_config_path = os.path.join(
        get_package_share_directory("lidar_localization_ros2"),
        "rviz",
        "localization.rviz",
    )

    # Path to the lidar localization launch file
    lidar_localization_launch_path = os.path.join(
        "/home/jacart2/dev_ws/src/ai-navigation/cart_control/localization_launch/launch",
        "lidar_localization.launch.py",
    )

    return LaunchDescription(
        [
            # Play the rosbag
            # ExecuteProcess(cmd=["ros2", "bag", "play", rosbag_path], output="screen"),
            # Launch rviz2 with the specified config
            # Include the lidar localization launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_localization_launch_path)
            ),
            ExecuteProcess(cmd=["rviz2", "-d", rviz_config_path], output="screen"),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
