from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    mock_bringup = PathJoinSubstitution(
        [FindPackageShare("humaware_launch"), "launch", "mock_bringup.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id",
                default_value="mock_001",
                description="Robot namespace and runtime identifier.",
            ),
            LogInfo(msg="Starting HumaWare simulation navigation demo scaffold"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mock_bringup),
                launch_arguments={"robot_id": robot_id}.items(),
            ),
        ]
    )
