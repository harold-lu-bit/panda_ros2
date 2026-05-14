from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_id_param_name = "arm_id"
    target_rate_param_name = "target_rate"
    arm_id_arg = DeclareLaunchArgument(
        arm_id_param_name,
        default_value="panda",
        description="Name of the robot.",
    )
    target_rate_arg = DeclareLaunchArgument(
        target_rate_param_name,
        default_value="50.0",
        description="Target frequency (Hz) for throttled topics",
    )

    target_rate = LaunchConfiguration(target_rate_param_name)
    arm_id = LaunchConfiguration(arm_id_param_name)

    return LaunchDescription(
        [
            arm_id_arg,
            target_rate_arg,
            Node(
                package="topic_tools",
                executable="throttle",
                name="pose_throttler",
                output="screen",
                arguments=[
                    "messages",
                    ["/", arm_id, "/franka_robot_state_broadcaster/current_pose"],
                    target_rate,
                    [
                        "/",
                        arm_id,
                        "/franka_robot_state_broadcaster/current_pose_throttled",
                    ],
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="joint_state_throttler",
                output="screen",
                arguments=[
                    "messages",
                    ["/", arm_id, "/panda_arm/joint_states"],
                    target_rate,
                    ["/", arm_id, "/panda_arm/joint_states_throttled"],
                ],
            ),
        ]
    )
