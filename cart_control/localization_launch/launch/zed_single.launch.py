import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer


'''
Madison Front Camera with camera flip and IMU fusion:
ros2 launch localization_launch zed_single.launch.py \
  camera_name:=zed_front \
  camera_model:=zed2i \
  serial_number:=38667373 \
  camera_flip:=true \
  publish_urdf:=true
'''

def launch_setup(context, *args, **kwargs):
    actions = []
    
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')
    publish_urdf = LaunchConfiguration('publish_urdf')
    camera_flip = LaunchConfiguration('camera_flip')
    
    # Create component container
    container = ComposableNodeContainer(
        name='zed_container',
        namespace='zed',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
    )
    actions.append(container)
    
    # Include single camera launch and forward camera flip setting.
    zed_camera_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('localization_launch'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'serial_number': serial_number,
            'camera_id': camera_id,
            'publish_urdf': publish_urdf,
            'camera_flip': camera_flip,
            'container_name': 'zed_container',
            'namespace': 'zed',
        }.items(),
    )
    actions.append(zed_camera_launch)
    
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed_front',
            description='The name of the camera (used as namespace prefix)',
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2i',
            description='The model of the camera (zed, zedm, zed2, zed2i, zedx, zedxm)',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm'],
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value='0',
            description='Serial number of the camera to open',
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='-1',
            description='Camera ID (used if serial_number is 0)',
        ),
        DeclareLaunchArgument(
            'publish_urdf',
            default_value='true',
            description='Publish URDF and TF',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'camera_flip',
            default_value='false',
            description='Enable upside-down camera mounting compensation in ZED SDK.',
            choices=['true', 'false'],
        ),
        OpaqueFunction(function=launch_setup)
    ])