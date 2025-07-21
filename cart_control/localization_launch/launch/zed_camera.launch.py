# Copyright 2024 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)


def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    arr = str.split(',')

    return arr


def launch_setup(context, *args, **kwargs):
    return_array = []

    # Launch configuration variables - only keeping essential ones
    node_log_type = LaunchConfiguration('node_log_type')
    container_name = LaunchConfiguration('container_name')
    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    node_name = LaunchConfiguration('node_name')
    config_file = LaunchConfiguration('config_file')
    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')
    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    node_log_type_val = node_log_type.perform(context)
    container_name_val = container_name.perform(context)
    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)

    if(node_log_type_val == 'both'):
        node_log_effective = 'both'
    else:  # 'screen' or 'log'
        node_log_effective = {
            'stdout': node_log_type_val,
            'stderr': node_log_type_val
            }

    if (camera_name_val == ''):
        camera_name_val = 'zed'
    
    if(namespace_val == ''):
        namespace_val = camera_name_val
    else:
        node_name_val = camera_name_val
    
    # Camera configuration file path
    config_file_path = config_file.perform(context)
    info = 'Using configuration file: ' + config_file_path
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # Robot State Publisher node
    rsp_name = camera_name_val + '_state_publisher'
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output=node_log_effective,
        parameters=[{
            'use_sim_time': False,
            'robot_description': Command(['xacro ', xacro_path.perform(context), 
                                         ' camera_name:=', camera_name_val,
                                         ' camera_model:=', camera_model_val])
        }]
    )
    return_array.append(rsp_node)

    # ROS 2 Component Container
    if(container_name_val == ''):
        container_name_val='zed_container'
        distro = os.environ['ROS_DISTRO']
        if distro == 'foxy':
            # Foxy does not support the isolated mode
            container_exec='component_container'
        else:
            container_exec='component_container_isolated'
        
        zed_container = ComposableNodeContainer(
                name=container_name_val,
                namespace=namespace_val,
                package='rclcpp_components',
                executable=container_exec,
                arguments=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info'],
                output=node_log_effective,
                composable_node_descriptions=[]
        )
        return_array.append(zed_container)

    # ZED Node parameters
    node_parameters = [
            # YAML file - using only one config file now
            config_file_path
    ]

    node_parameters.append( 
            # Launch arguments must override the YAML files values - only keeping essentials
            {
                'general.camera_name': camera_name_val,
                'general.camera_model': camera_model_val,
                'general.serial_number': serial_number,
                'general.camera_id': camera_id,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'sensors.publish_imu_tf': publish_imu_tf
            }
    )

    # ZED Wrapper component
    if( camera_model_val=='zed' or
        camera_model_val=='zedm' or
        camera_model_val=='zed2' or
        camera_model_val=='zed2i' or
        camera_model_val=='zedx' or
        camera_model_val=='zedxm' or
        camera_model_val=='virtual'):
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCamera',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    else: # 'zedxonegs' or 'zedxone4k')
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCameraOne',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    
    full_container_name = '/' + namespace_val + '/' + container_name_val
    info = 'Loading ZED node `' + node_name_val + '` in container `' + full_container_name + '`'
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_wrapper_component]
    )
    return_array.append(load_composable_node)

    return return_array

def generate_launch_description():
    return LaunchDescription(
        [
            # Declare essential launch arguments
            DeclareLaunchArgument(
                'node_log_type',
                default_value=TextSubstitution(text='both'),
                description='The log type of the node: `screen`, `log` or `both`.',
                choices=['screen', 'log', 'both']),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It will be used as node namespace.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']),
            DeclareLaunchArgument(
                'container_name',
                default_value='',
                description='The name of the container to be used to load the ZED component. If empty (default) a new container will be created.'),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='The namespace of the node. If empty (default) the camera name is used.'),
            DeclareLaunchArgument(
                'node_name',
                default_value='zed_node',
                description='The name of the zed_wrapper node.'),
            DeclareLaunchArgument(
                'config_file',
                default_value=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'zed_config.yaml'),
                description='Path to the YAML configuration file for the camera.'),
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='The serial number of the camera to be opened.'),
            DeclareLaunchArgument(
                'camera_id',
                default_value='-1',
                description='The ID of the camera to be opened.'),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and starts Robot State Publisher.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='true',
                description='Enable publication of the `odom -> camera_link` TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_map_tf',
                default_value='true',
                description='Enable publication of the `map -> odom` TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_imu_tf',
                default_value='false',
                description='Enable publication of the IMU TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF file as a xacro file.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
