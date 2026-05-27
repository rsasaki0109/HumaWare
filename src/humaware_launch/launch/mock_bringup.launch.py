from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    enable_keyboard_teleop = LaunchConfiguration("enable_keyboard_teleop")
    enable_nav2_bridge = LaunchConfiguration("enable_nav2_bridge")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id",
                default_value="mock_001",
                description="Robot namespace and runtime identifier.",
            ),
            DeclareLaunchArgument(
                "enable_keyboard_teleop",
                default_value="false",
                description="Start keyboard teleop provider in this launch process.",
            ),
            DeclareLaunchArgument(
                "enable_nav2_bridge",
                default_value="false",
                description="Start Nav2-style velocity bridge in this launch process.",
            ),
            LogInfo(msg=["Starting HumaWare mock bringup for ", robot_id]),
            Node(
                package="humaware_mode_manager",
                executable="mode_manager_node",
                namespace=robot_id,
                name="mode_manager",
                output="screen",
                parameters=[{"robot_id": robot_id}],
            ),
            Node(
                package="humaware_mock_robot",
                executable="mock_robot_node",
                namespace=robot_id,
                name="mock_robot",
                output="screen",
                parameters=[{"robot_id": robot_id}],
            ),
            Node(
                package="humaware_safety_manager",
                executable="safety_manager_node",
                namespace=robot_id,
                name="safety_manager",
                output="screen",
                parameters=[{"robot_id": robot_id}],
            ),
            Node(
                package="humaware_mock_locomotion_adapter",
                executable="mock_locomotion_adapter_node",
                namespace=robot_id,
                name="mock_locomotion_adapter",
                output="screen",
                parameters=[{"robot_id": robot_id}],
            ),
            Node(
                package="humaware_command_arbiter",
                executable="command_arbiter_node",
                namespace=robot_id,
                name="command_arbiter",
                output="screen",
                parameters=[{"robot_id": robot_id}],
            ),
            Node(
                package="humaware_diagnostics_aggregator",
                executable="diagnostics_aggregator_node",
                namespace=robot_id,
                name="diagnostics_aggregator",
                output="screen",
                parameters=[{"robot_id": robot_id}],
            ),
            Node(
                package="humaware_keyboard_teleop",
                executable="keyboard_teleop_node",
                namespace=robot_id,
                name="keyboard_teleop",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(enable_keyboard_teleop),
                parameters=[{"robot_id": robot_id}],
            ),
            Node(
                package="humaware_nav2_bridge",
                executable="nav2_bridge_node",
                namespace=robot_id,
                name="nav2_bridge",
                output="screen",
                condition=IfCondition(enable_nav2_bridge),
                parameters=[{"robot_id": robot_id}],
            ),
        ]
    )
