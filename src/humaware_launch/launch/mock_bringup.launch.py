from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    enable_keyboard_teleop = LaunchConfiguration("enable_keyboard_teleop")
    enable_nav2_bridge = LaunchConfiguration("enable_nav2_bridge")
    teleop_heartbeat_timeout_s = LaunchConfiguration("teleop_heartbeat_timeout_s")
    teleop_heartbeat_timeout_triggers_mrm = LaunchConfiguration(
        "teleop_heartbeat_timeout_triggers_mrm"
    )
    require_hardware_heartbeat = LaunchConfiguration("require_hardware_heartbeat")
    hardware_heartbeat_timeout_s = LaunchConfiguration("hardware_heartbeat_timeout_s")
    hardware_heartbeat_timeout_triggers_mrm = LaunchConfiguration(
        "hardware_heartbeat_timeout_triggers_mrm"
    )

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
            DeclareLaunchArgument(
                "teleop_heartbeat_timeout_s",
                default_value="1.0",
                description=(
                    "Teleop heartbeat watchdog timeout for the safety manager."
                ),
            ),
            DeclareLaunchArgument(
                "teleop_heartbeat_timeout_triggers_mrm",
                default_value="false",
                description=(
                    "When true, a teleop heartbeat timeout auto-triggers an MRM "
                    "instead of only raising a warning."
                ),
            ),
            DeclareLaunchArgument(
                "require_hardware_heartbeat",
                default_value="false",
                description=(
                    "When true, the safety manager watches the hardware "
                    "heartbeat while in an active mode."
                ),
            ),
            DeclareLaunchArgument(
                "hardware_heartbeat_timeout_s",
                default_value="1.0",
                description=(
                    "Hardware heartbeat watchdog timeout for the safety manager."
                ),
            ),
            DeclareLaunchArgument(
                "hardware_heartbeat_timeout_triggers_mrm",
                default_value="true",
                description=(
                    "When true, a hardware heartbeat timeout auto-triggers an MRM "
                    "instead of only raising a warning."
                ),
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
                parameters=[
                    {
                        "robot_id": robot_id,
                        "teleop_heartbeat_timeout_s": ParameterValue(
                            teleop_heartbeat_timeout_s, value_type=float
                        ),
                        "teleop_heartbeat_timeout_triggers_mrm": ParameterValue(
                            teleop_heartbeat_timeout_triggers_mrm, value_type=bool
                        ),
                        "require_hardware_heartbeat": ParameterValue(
                            require_hardware_heartbeat, value_type=bool
                        ),
                        "hardware_heartbeat_timeout_s": ParameterValue(
                            hardware_heartbeat_timeout_s, value_type=float
                        ),
                        "hardware_heartbeat_timeout_triggers_mrm": ParameterValue(
                            hardware_heartbeat_timeout_triggers_mrm, value_type=bool
                        ),
                    }
                ],
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
