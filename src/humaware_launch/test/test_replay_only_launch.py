"""Launch test for the replay-only profile.

The replay-only profile must publish runtime decision topics but must not
start any hardware adapter, mock robot, or mock locomotion adapter node.
"""

import os
import time
import unittest

from humaware_msgs.msg import (
    CommandArbitrationState,
    HealthState,
    ModeState,
    SafetyState,
)
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
import rclpy


ROBOT_ID = f"replay_only_{os.getpid()}"

FORBIDDEN_NODES = (
    "mock_robot",
    "mock_locomotion_adapter",
    "keyboard_teleop",
    "nav2_bridge",
)

EXPECTED_RUNTIME_NODES = (
    "mode_manager",
    "capability_registry",
    "skill_server",
    "safety_manager",
    "command_arbiter",
    "diagnostics_aggregator",
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_file = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "launch", "replay_only.launch.py")
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={"robot_id": ROBOT_ID}.items(),
    )

    return launch.LaunchDescription([bringup, launch_testing.actions.ReadyToTest()]), {
        "robot_id": ROBOT_ID,
    }


class TestReplayOnlyLaunch(unittest.TestCase):
    def test_runtime_topics_publish_without_hardware(self, robot_id):
        topics = [
            (f"/{robot_id}/mode/state", ModeState),
            (f"/{robot_id}/safety/state", SafetyState),
            (f"/{robot_id}/runtime/health", HealthState),
            (f"/{robot_id}/runtime/command_arbitration_state", CommandArbitrationState),
        ]

        with WaitForTopics(topics, timeout=15.0, messages_received_buffer_length=2) as waiter:
            self.assertEqual(set(), waiter.topics_not_received())

    def test_no_hardware_adapter_nodes_are_launched(self, robot_id):
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(f"replay_only_discovery_{os.getpid()}", context=context)
        try:
            deadline = time.time() + 10.0
            expected = {f"/{robot_id}/{name}" for name in EXPECTED_RUNTIME_NODES}
            seen = set()
            while time.time() < deadline:
                full_names = {
                    f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
                    for name, namespace in node.get_node_names_and_namespaces()
                }
                seen = full_names
                if expected.issubset(full_names):
                    break
                time.sleep(0.2)

            missing = expected - seen
            self.assertFalse(
                missing,
                f"replay-only profile failed to launch runtime nodes: {missing}",
            )

            forbidden_present = {
                full
                for full in seen
                for forbidden in FORBIDDEN_NODES
                if full == f"/{robot_id}/{forbidden}"
            }
            self.assertFalse(
                forbidden_present,
                f"replay-only profile launched forbidden hardware nodes: {forbidden_present}",
            )
        finally:
            node.destroy_node()
            rclpy.shutdown(context=context)


@launch_testing.post_shutdown_test()
class TestReplayOnlyLaunchShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
