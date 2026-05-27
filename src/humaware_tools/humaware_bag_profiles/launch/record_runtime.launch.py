from datetime import datetime, timezone

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _runtime_topics(robot_id: str) -> list[str]:
    robot = f"/{robot_id}"
    return [
        f"{robot}/mode/state",
        f"{robot}/mode/transition_state",
        f"{robot}/safety/state",
        f"{robot}/safety/mrm_state",
        f"{robot}/runtime/health",
        f"{robot}/capabilities",
        f"{robot}/skills/state",
        f"{robot}/locomotion/state",
        f"{robot}/runtime/command_arbitration_state",
        "/diagnostics",
        f"{robot}/navigation/nav2_bridge_state",
        f"{robot}/teleop/heartbeat",
        f"{robot}/cmd_vel/approved",
        f"{robot}/teleop/cmd_vel",
        f"{robot}/autonomy/cmd_vel",
        f"{robot}/policy/cmd_vel",
        "/tf",
        "/tf_static",
        "/clock",
    ]


def _record_process(context):
    robot_id = LaunchConfiguration("robot_id").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context)
    storage_id = LaunchConfiguration("storage_id").perform(context)
    all_topics = LaunchConfiguration("all_topics").perform(context).lower() == "true"

    if not output_dir:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output_dir = f"artifacts/bags/{robot_id}_{timestamp}"

    cmd = ["ros2", "bag", "record", "--output", output_dir, "--storage", storage_id]
    if all_topics:
        cmd.append("--all-topics")
    else:
        cmd.append("--topics")
        cmd.extend(_runtime_topics(robot_id))

    return [
        LogInfo(msg=["Recording HumaWare runtime bag to ", output_dir]),
        ExecuteProcess(cmd=cmd, output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id",
                default_value="mock_001",
                description="Robot namespace and runtime identifier.",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="",
                description="Output bag directory. Defaults to artifacts/bags/<robot>_<utc>.",
            ),
            DeclareLaunchArgument(
                "storage_id",
                default_value="sqlite3",
                description="rosbag2 storage backend.",
            ),
            DeclareLaunchArgument(
                "all_topics",
                default_value="false",
                description="Record every topic instead of the runtime profile.",
            ),
            OpaqueFunction(function=_record_process),
        ]
    )
