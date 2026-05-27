import os
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from humaware_msgs.msg import HealthState, LocomotionState, ModeState, SafetyState
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest


ROBOT_ID = f"mock_launch_test_{os.getpid()}"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_file = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "launch", "mock_bringup.launch.py")
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "robot_id": ROBOT_ID,
            "enable_keyboard_teleop": "false",
            "enable_nav2_bridge": "false",
        }.items(),
    )

    return launch.LaunchDescription([bringup, launch_testing.actions.ReadyToTest()]), {
        "robot_id": ROBOT_ID,
    }


class TestMockBringupLaunch(unittest.TestCase):
    def test_runtime_topics_publish(self, robot_id):
        topics = [
            (f"/{robot_id}/mode/state", ModeState),
            (f"/{robot_id}/safety/state", SafetyState),
            (f"/{robot_id}/locomotion/state", LocomotionState),
            (f"/{robot_id}/runtime/health", HealthState),
            ("/diagnostics", DiagnosticArray),
        ]

        with WaitForTopics(topics, timeout=20.0, messages_received_buffer_length=10) as waiter:
            self.assertEqual(set(), waiter.topics_not_received())

            mode = self._latest_message(waiter, f"/{robot_id}/mode/state")
            safety = self._latest_message(waiter, f"/{robot_id}/safety/state")
            locomotion = self._latest_message(waiter, f"/{robot_id}/locomotion/state")
            health = self._latest_message(waiter, f"/{robot_id}/runtime/health")
            diagnostics = waiter.received_messages("/diagnostics")

            self.assertEqual(robot_id, mode.robot_id)
            self.assertEqual(robot_id, safety.robot_id)
            self.assertEqual(robot_id, locomotion.robot_id)
            self.assertEqual(robot_id, health.robot_id)
            self.assertTrue(
                any(
                    status.name == f"{robot_id}/runtime_health"
                    for message in diagnostics
                    for status in message.status
                ),
                "diagnostics did not include the runtime health status",
            )

    @staticmethod
    def _latest_message(waiter, topic):
        messages = waiter.received_messages(topic)
        assert messages, f"no messages received on {topic}"
        return messages[-1]


@launch_testing.post_shutdown_test()
class TestMockBringupShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
