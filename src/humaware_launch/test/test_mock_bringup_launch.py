import os
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from humaware_msgs.msg import (
    CapabilityRegistry,
    HealthState,
    LocomotionState,
    ModeState,
    SafetyState,
)
from humaware_msgs.srv import ListCapabilities
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


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
            (f"/{robot_id}/capabilities", CapabilityRegistry),
            ("/diagnostics", DiagnosticArray),
        ]

        with WaitForTopics(topics, timeout=20.0, messages_received_buffer_length=10) as waiter:
            self.assertEqual(set(), waiter.topics_not_received())

            mode = self._latest_message(waiter, f"/{robot_id}/mode/state")
            safety = self._latest_message(waiter, f"/{robot_id}/safety/state")
            locomotion = self._latest_message(waiter, f"/{robot_id}/locomotion/state")
            health = self._latest_message(waiter, f"/{robot_id}/runtime/health")
            capabilities = self._latest_message(waiter, f"/{robot_id}/capabilities")
            diagnostics = waiter.received_messages("/diagnostics")

            self.assertEqual(robot_id, mode.robot_id)
            self.assertEqual(robot_id, safety.robot_id)
            self.assertEqual(robot_id, locomotion.robot_id)
            self.assertEqual(robot_id, health.robot_id)
            self.assertEqual(robot_id, capabilities.robot_id)
            self.assertIn("stop", {capability.name for capability in capabilities.capabilities})
            self.assertIn(
                "walk_velocity",
                {capability.name for capability in capabilities.capabilities},
            )
            self.assertTrue(
                any(
                    status.name == f"{robot_id}/runtime_health"
                    for message in diagnostics
                    for status in message.status
                ),
                "diagnostics did not include the runtime health status",
            )

    def test_capability_service_lists_selected_names(self, robot_id):
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(f"capability_registry_test_{os.getpid()}", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        try:
            client = node.create_client(ListCapabilities, f"/{robot_id}/capabilities/list")
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))

            request = ListCapabilities.Request()
            request.names = ["stop", "walk_velocity", "missing_capability"]
            future = client.call_async(request)

            deadline = time.time() + 10.0
            while time.time() < deadline and not future.done():
                executor.spin_once(timeout_sec=0.1)

            self.assertTrue(future.done(), "capability registry service did not respond")
            response = future.result()
            self.assertEqual(["missing_capability"], list(response.missing_names))
            self.assertEqual(
                {"stop", "walk_velocity"},
                {capability.name for capability in response.capabilities},
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    @staticmethod
    def _latest_message(waiter, topic):
        messages = waiter.received_messages(topic)
        assert messages, f"no messages received on {topic}"
        return messages[-1]


@launch_testing.post_shutdown_test()
class TestMockBringupShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
