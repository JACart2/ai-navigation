import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mola_autonomy_launch = os.path.join(
        get_package_share_directory("cart_launch"),
        "launch",
        "mola_autonomy.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mola_autonomy_launch),
                launch_arguments={"cart": "james"}.items(),
            )
        ]
    )
