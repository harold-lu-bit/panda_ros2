import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_arm_id_parameter_name = "left_arm_id"
    right_arm_id_parameter_name = "right_arm_id"
    left_ip_parameter_name = "left_ip"
    right_ip_parameter_name = "right_ip"
    load_gripper_parameter_name = "load_gripper"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    use_rviz_parameter_name = "use_dual_rviz"

    left_arm_id = LaunchConfiguration(left_arm_id_parameter_name)
    right_arm_id = LaunchConfiguration(right_arm_id_parameter_name)
    left_ip = LaunchConfiguration(left_ip_parameter_name)
    right_ip = LaunchConfiguration(right_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    rviz_path = (
        Path(get_package_share_directory("franka_description"))
        / "rviz"
        / "dual_franka.rviz"
    )
    extrinsics_path = (
        Path(get_package_share_directory("franka_bringup"))
        / "config"
        / "dual_arm_extrinsics.yaml"
    )

    if not extrinsics_path.exists():
        return LaunchDescription(
            [
                LogInfo(
                    msg=f"Extrinsics file not found. Please check the path: {extrinsics_path}"
                )
            ]
        )
    try:
        extrin_config = yaml.safe_load(extrinsics_path.read_text())
        trans = extrin_config["translation"]
        orient = extrin_config["orientation"]
    except Exception as e:
        return LaunchDescription(
            [
                LogInfo(
                    msg=f"Error loading extrinsics file and frames: {e}"
                )
            ]
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                left_arm_id_parameter_name,
                default_value="panda_left",
                description="Name of the left arm.",
            ),
            DeclareLaunchArgument(
                right_arm_id_parameter_name,
                default_value="panda_right",
                description="Name of the right arm.",
            ),
            DeclareLaunchArgument(
                left_ip_parameter_name,
                description="Hostname or IP address of the left arm.",
            ),
            DeclareLaunchArgument(
                right_ip_parameter_name,
                description="Hostname or IP address of the left arm.",
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value="false",
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
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='dual_franka_extrinsics_publisher',
                arguments=[
                    '--x', f'{trans["x"]}',
                    '--y', f'{trans["y"]}',
                    '--z', f'{trans["z"]}',
                    '--qx', f'{orient["qx"]}',
                    '--qy', f'{orient["qy"]}',
                    '--qz', f'{orient["qz"]}',
                    '--qw', f'{orient["qw"]}',
                    '--frame-id', [left_arm_id, "_link0"],
                    '--child-frame-id', [right_arm_id, "_link0"]
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_bringup"),
                                "launch",
                                "cartesian_impedance_example_controller.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "arm_id": left_arm_id,
                    "robot_ip": left_ip,
                    "use_arm_id_as_ns": "true",
                    "use_rviz": "false",
                    load_gripper_parameter_name: load_gripper,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    fake_sensor_commands_parameter_name: fake_sensor_commands,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_bringup"),
                                "launch",
                                "cartesian_impedance_example_controller.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "arm_id": right_arm_id,
                    "robot_ip": right_ip,
                    "use_arm_id_as_ns": "true",
                    "use_rviz": "false",
                    load_gripper_parameter_name: load_gripper,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    fake_sensor_commands_parameter_name: fake_sensor_commands,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_path.as_posix()],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
