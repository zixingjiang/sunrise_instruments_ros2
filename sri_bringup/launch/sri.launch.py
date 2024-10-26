# Copyright 2024 Zixing Jiang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    sensor_name = LaunchConfiguration('sensor_name')
    sensor_ip = LaunchConfiguration('sensor_ip')
    sensor_port = LaunchConfiguration('sensor_port')
    gui = LaunchConfiguration('gui')

    tf_prefix = PythonExpression(["'", sensor_name, "_' if '", sensor_name, "' != '' else ''"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('sri_bringup'),
                    'urdf',
                    'sri_ft_sensor_setup.urdf.xacro',
                ]
            ),
            ' sensor_name:=', sensor_name,
            ' sensor_ip:=', sensor_ip,
            ' sensor_port:=', sensor_port,
        ]
    )
    robot_description = \
        {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controllers_config = PathJoinSubstitution(
        [
            FindPackageShare('sri_bringup'),
            'config',
            'sri_controller_manager.yaml',
        ]
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare('sri_bringup'),
            'rviz',
            'sri.rviz',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ParameterFile(controllers_config, allow_substs=True)],
        output='screen',
    )

    sri_fts_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'sri_fts_broadcaster',
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(gui),
    )

    nodes = [
        controller_manager_node,
        node_robot_state_publisher,
        sri_fts_broadcaster_spawner,
        rviz_node,
    ] 
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_name',
            default_value='',
            description='name of the sensor'),
        DeclareLaunchArgument(
            'sensor_ip',
            default_value='0.0.0.0',
            description='IP address of the sensor'),
        DeclareLaunchArgument(
            'sensor_port',
            default_value='4008',
            description='Port of the sensor'),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Use RViz GUI for visualization'
        ),
        DeclareLaunchArgument(
           'tf_prefix',
           default_value=tf_prefix,
           description='Prefix for the node names'
        )
        ] + nodes)