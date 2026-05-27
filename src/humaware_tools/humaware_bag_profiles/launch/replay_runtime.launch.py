from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _replay_process(context):
    bag_path = LaunchConfiguration("bag_path").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    publish_clock = LaunchConfiguration("publish_clock").perform(context).lower() == "true"
    loop = LaunchConfiguration("loop").perform(context).lower() == "true"

    cmd = ["ros2", "bag", "play", bag_path, "--rate", rate]
    if publish_clock:
        cmd.append("--clock")
    if loop:
        cmd.append("--loop")

    return [
        LogInfo(
            msg=[
                "Replaying HumaWare runtime bag from ",
                bag_path,
                ". Do not connect this replay to real hardware command adapters.",
            ]
        ),
        ExecuteProcess(cmd=cmd, output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                description="Path to a rosbag2 directory.",
            ),
            DeclareLaunchArgument(
                "rate",
                default_value="1.0",
                description="Playback rate.",
            ),
            DeclareLaunchArgument(
                "publish_clock",
                default_value="true",
                description="Publish /clock during replay.",
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="false",
                description="Loop playback.",
            ),
            OpaqueFunction(function=_replay_process),
        ]
    )
