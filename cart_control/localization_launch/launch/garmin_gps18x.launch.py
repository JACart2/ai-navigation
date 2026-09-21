from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    frame_id = LaunchConfiguration("frame_id")
    use_rmc = LaunchConfiguration("use_rmc")
    time_ref_source = LaunchConfiguration("time_ref_source")

    gps_driver = Node(
        package="localization_launch",
        executable="garmin_gps18x_driver",
        name="garmin_gps18x_driver",
        output="screen",
        parameters=[
            {
                "port": port,
                "baud": baud,
                "frame_id": frame_id,
                "use_rmc": use_rmc,
                "time_ref_source": time_ref_source,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Use loopback-only DDS traffic so GPS topics survive WiFi link loss",
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyACM0",
                description="Serial port for the Garmin GPS 18x USB",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="4800",
                description="Serial baud rate for the GPS",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="gps",
                description="TF frame attached to the GPS antenna",
            ),
            DeclareLaunchArgument(
                "use_rmc",
                default_value="true",
                description="Use RMC sentences if no GGA fix is available",
            ),
            DeclareLaunchArgument(
                "time_ref_source",
                default_value="gps",
                description="Source label for time reference messages",
            ),
            gps_driver,
        ]
    )