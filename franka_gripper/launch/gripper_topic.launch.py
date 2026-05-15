# Copyright (c) 2026
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
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    namespace_parameter_name = 'namespace'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    input_topic_parameter_name = 'input_topic'
    output_topic_parameter_name = 'output_topic'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    input_topic = LaunchConfiguration(input_topic_parameter_name)
    output_topic = LaunchConfiguration(output_topic_parameter_name)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ip_parameter_name, description='Hostname or IP address of the robot.'
            ),
            DeclareLaunchArgument(
                namespace_parameter_name,
                default_value='',
                description='Namespace for the gripper.',
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value='false',
                description='Do not connect to a real gripper when fake hardware is enabled.',
            ),
            DeclareLaunchArgument(
                input_topic_parameter_name,
                default_value='gripper/openness_command',
                description='Input gripper openness topic.',
            ),
            DeclareLaunchArgument(
                output_topic_parameter_name,
                default_value='gripper/openness',
                description='Output discrete gripper openness topic.',
            ),
            Node(
                package='franka_gripper',
                executable='gripper_topic_node',
                namespace=namespace,
                name='panda_gripper_topic',
                parameters=[
                    {
                        'robot_ip': robot_ip,
                        'input_topic': input_topic,
                        'output_topic': output_topic,
                    }
                ],
                output='screen',
                condition=UnlessCondition(use_fake_hardware),
            ),
        ]
    )
