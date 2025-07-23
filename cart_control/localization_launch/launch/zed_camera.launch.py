import os
 
from ament_index_python.packages import get_package_share_directory
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
 
def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_model = LaunchConfiguration('camera_model').perform(context)
 
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
 
    # Path to common stereo config
    config_common_path = os.path.join(zed_wrapper_dir, 'config', 'common.yaml')
    # Path to the camera-specific config
    config_camera_path = os.path.join(zed_wrapper_dir, 'config', f'{camera_model}.yaml')
    # Path to URDF
    xacro_path = os.path.join(zed_wrapper_dir, 'urdf', 'zed_descr.urdf.xacro')
 
    # Robot State Publisher node for URDF
    rsp_node = Node(
        package='robot_state_publisher',
        namespace=camera_name,
        executable='robot_state_publisher',
        name=f'{camera_name}_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', xacro_path,
                ' camera_name:=', camera_name,
                ' camera_model:=', camera_model
            ])
        }]
    )
 
    # Component container for ZED node
    container = ComposableNodeContainer(
        name='zed_container',
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        composable_node_descriptions=[],
    )
 
    # ZED node
    zed_node = LoadComposableNodes(
        target_container=f'/{camera_name}/zed_container',
        composable_node_descriptions=[
            ComposableNode(
                package='zed_components',
                namespace=camera_name,
                plugin='stereolabs::ZedCamera',
                name='zed_node',
                parameters=[
                    config_common_path,
                    config_camera_path,
                    {'general.camera_name': camera_name,
                     'general.camera_model': camera_model}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
 
    return [rsp_node, container, zed_node]
 
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value=TextSubstitution(text='zed'),
            description='The name of the camera (used as namespace).'
        ),
        DeclareLaunchArgument(
            'camera_model',
            description='The model of the camera.',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']
        ),
        OpaqueFunction(function=launch_setup)
    ])
