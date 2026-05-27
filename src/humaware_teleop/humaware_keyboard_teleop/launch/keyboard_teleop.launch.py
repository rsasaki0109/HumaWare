from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id",
                default_value="mock_001",
                description="Robot namespace and runtime identifier.",
            ),
            LogInfo(msg=["Starting HumaWare keyboard teleop for ", robot_id]),
            Node(
                package="humaware_keyboard_teleop",
                executable="keyboard_teleop_node",
                namespace=robot_id,
                name="keyboard_teleop",
                output="screen",
                emulate_tty=True,
                parameters=[{"robot_id": robot_id}],
            ),
        ]
    )
