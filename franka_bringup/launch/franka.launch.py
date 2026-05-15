#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_id_parameter_name = 'arm_id'
    robot_ip_parameter_name = 'robot_ip'
    use_arm_id_as_ns_parameter_name = 'use_arm_id_as_ns'
    load_gripper_parameter_name = 'load_gripper'
    start_gripper_action_server_parameter_name = 'start_gripper_action_server'
    controllers_file_parameter_name = 'controllers_file'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'
    publish_description_name = 'publish_description'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    start_gripper_action_server = LaunchConfiguration(start_gripper_action_server_parameter_name)
    use_arm_id_as_ns = LaunchConfiguration(use_arm_id_as_ns_parameter_name)
    controllers_file = LaunchConfiguration(controllers_file_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    publish_description = LaunchConfiguration(publish_description_name)
    namespace = PythonExpression(["'/' + '", arm_id, "' if '",
                                  use_arm_id_as_ns, "' == 'true' else ''"])

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' arm_id:=', arm_id,
         ' hand:=', load_gripper, ' robot_ip:=', robot_ip,
         ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare('franka_bringup'),
            'config',
            controllers_file,
        ]
    )
    joint_name_postfix = '_finger_joint'
    joint_names = [
        '[',
        arm_id,
        joint_name_postfix,
        '1',
        ',',
        arm_id,
        joint_name_postfix,
        '2',
        ']',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value='panda',
            description='Name of the robot.'),
        DeclareLaunchArgument(
            use_arm_id_as_ns_parameter_name,
            default_value='true',
            description='Use arm_id as namespace.'),
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            controllers_file_parameter_name,
            default_value='controllers.yaml',
            description='YAML file with the controllers configuration in config folder.'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            start_gripper_action_server_parameter_name,
            default_value='true',
            description='Start the legacy action-based Franka gripper node.'),
        DeclareLaunchArgument(
            publish_description_name,
            default_value='true',
            description='Publish the robot description through topic.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(publish_description),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=namespace,
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['panda_arm/joint_states', 'panda_gripper/joint_states'],
                 'rate': 30}],
            condition=IfCondition(publish_description)
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[franka_controllers,
                        {'robot_description': robot_description},
                        {'arm_id': arm_id},
                        {'load_gripper': load_gripper},
                        ],
            remappings=[('joint_states', ['panda_arm/joint_states'])],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', [namespace, '/controller_manager']],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster', '-c', [namespace, '/controller_manager']],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={
                arm_id_parameter_name: arm_id,
                robot_ip_parameter_name: robot_ip,
                use_fake_hardware_parameter_name: use_fake_hardware,
                "namespace": namespace,
                "joint_names": joint_names,
            }.items(),
            condition=IfCondition(PythonExpression(
                ["'", load_gripper, "' == 'true' and '", start_gripper_action_server, "' == 'true'"]
            ))
        ),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    ])
