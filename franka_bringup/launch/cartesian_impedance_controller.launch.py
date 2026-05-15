from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_id_parameter_name = "arm_id"
    robot_ip_parameter_name = "robot_ip"
    use_arm_id_as_ns_parameter_name = "use_arm_id_as_ns"
    load_gripper_parameter_name = "load_gripper"
    start_gripper_action_server_parameter_name = "start_gripper_action_server"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    use_rviz_parameter_name = "use_rviz"
    publish_description_name = "publish_description"
    use_throttle_parameter_name = "use_throttle"
    throttle_rate_parameter_name = "throttle_rate"

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_arm_id_as_ns = LaunchConfiguration(use_arm_id_as_ns_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    publish_description = LaunchConfiguration(publish_description_name)
    use_throttle = LaunchConfiguration(use_throttle_parameter_name)
    throttle_rate = LaunchConfiguration(throttle_rate_parameter_name)
    namespace = PythonExpression(
        ["'/' + '", arm_id, "' if '", use_arm_id_as_ns, "' == 'true' else ''"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                arm_id_parameter_name,
                default_value="panda",
                description="Name of the robot.",
            ),
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                description="Hostname or IP address of the robot.",
            ),
            DeclareLaunchArgument(
                use_arm_id_as_ns_parameter_name,
                default_value="true",
                description="Use arm_id as namespace.",
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value="true",
                description="Visualize the robot in Rviz",
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value="false",
                description="Use fake hardware",
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value="false",
                description="Fake sensor commands. Only valid when '{}' is true".format(
                    use_fake_hardware_parameter_name
                ),
            ),
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value="true",
                description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded "
                "without an end-effector.",
            ),
            DeclareLaunchArgument(
                publish_description_name,
                default_value="true",
                description="Publish the robot description through topic.",
            ),
            DeclareLaunchArgument(
                use_throttle_parameter_name,
                default_value="true",
                description="Enable throttling of high-frequency topics.",
            ),
            DeclareLaunchArgument(
                throttle_rate_parameter_name,
                default_value="50.0",
                description="Target frequency (Hz) for throttled topics.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_bringup"),
                                "launch",
                                "franka.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    arm_id_parameter_name: arm_id,
                    robot_ip_parameter_name: robot_ip,
                    use_arm_id_as_ns_parameter_name: use_arm_id_as_ns,
                    load_gripper_parameter_name: load_gripper,
                    start_gripper_action_server_parameter_name: "false",
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    fake_sensor_commands_parameter_name: fake_sensor_commands,
                    use_rviz_parameter_name: use_rviz,
                    publish_description_name: publish_description,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_gripper"),
                                "launch",
                                "gripper_topic.launch.py",
                            ]
                        )
                    ]
                ),
                condition=IfCondition(load_gripper),
                launch_arguments={
                    robot_ip_parameter_name: robot_ip,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    "namespace": namespace,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_bringup"),
                                "launch",
                                "franka_throttle.launch.py",
                            ]
                        )
                    ]
                ),
                condition=IfCondition(use_throttle),
                launch_arguments={
                    arm_id_parameter_name: arm_id,
                    throttle_rate_parameter_name: throttle_rate,
                }.items(),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    "cartesian_impedance_controller",
                    "-c",
                    [namespace, "/controller_manager"],
                ],
                output="screen",
            ),
        ]
    )
