"""Replay-only HumaWare launch profile.

This profile starts the runtime decision graph (mode manager, capability
registry, skill server, safety manager, command arbiter, diagnostics
aggregator) without any hardware adapter, mock robot, or mock locomotion
adapter. It is intended for replaying rosbags through the runtime so that
mode transitions, safety state, and command arbitration can be inspected
without connecting to a real or simulated robot.

The profile must never include nodes that publish to hardware command
topics or that translate approved runtime commands into vendor commands.
"""

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
            LogInfo(
                msg=[
                    "Starting HumaWare replay-only profile for ",
                    robot_id,
                    " (no hardware adapters will be launched)",
                ]
            ),
            Node(
                package="humaware_mode_manager",
                executable="mode_manager_node",
                namespace=robot_id,
                name="mode_manager",
                output="screen",
                parameters=[{"robot_id": robot_id}],
            ),
            Node(
                package="humaware_capability_registry",
                executable="capability_registry_node",
                namespace=robot_id,
                name="capability_registry",
                output="screen",
                parameters=[{"robot_id": robot_id}],
            ),
            Node(
                package="humaware_skill_server",
                executable="skill_server_node",
                namespace=robot_id,
                name="skill_server",
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
        ]
    )
