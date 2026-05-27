"""End-to-end launch test for the scripted policy provider.

The test launches the mock bringup plus a scripted policy provider,
walks the runtime into ``MODE_AI_POLICY`` via ``TELEOP``, and verifies
that the candidate velocity published by the scripted provider is
approved by the command arbiter under ``SOURCE_AI_POLICY``.
"""

import os
import time
import unittest

from geometry_msgs.msg import TwistStamped
from humaware_msgs.msg import (
    CommandArbitrationState,
    ModeState,
    SafetyState,
)
from humaware_msgs.srv import SetMode
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


ROBOT_ID = f"mock_scripted_{os.getpid()}"
PLAN_LINEAR_X = 0.07
PLAN_ANGULAR_Z = 0.11


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    bringup_path = os.path.join(launch_dir, "launch", "mock_bringup.launch.py")
    plan_path = os.path.join(
        os.path.dirname(__file__), "fixtures", "scripted_policy_plan.yaml"
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_path),
        launch_arguments={
            "robot_id": ROBOT_ID,
            "enable_keyboard_teleop": "false",
            "enable_nav2_bridge": "false",
        }.items(),
    )

    scripted_provider = Node(
        package="humaware_policy_provider_scripted",
        executable="policy_provider_scripted_node",
        namespace=ROBOT_ID,
        name="policy_provider_scripted",
        output="screen",
        parameters=[
            {
                "robot_id": ROBOT_ID,
                "enabled": True,
                "plan_yaml_path": plan_path,
                "publish_rate_hz": 20.0,
            }
        ],
    )

    return launch.LaunchDescription(
        [bringup, scripted_provider, launch_testing.actions.ReadyToTest()]
    ), {"robot_id": ROBOT_ID}


class TestScriptedPolicyLaunch(unittest.TestCase):
    def test_scripted_candidate_is_approved_under_ai_policy(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/safety/state", SafetyState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"scripted_policy_launch_test_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        approved_commands: list = []
        arbitration_states: list = []
        try:
            node.create_subscription(
                TwistStamped,
                f"/{robot_id}/cmd_vel/approved",
                approved_commands.append,
                10,
            )
            node.create_subscription(
                CommandArbitrationState,
                f"/{robot_id}/runtime/command_arbitration_state",
                arbitration_states.append,
                10,
            )

            mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")
            self.assertTrue(
                mode_client.wait_for_service(timeout_sec=10.0),
                "mode/set service did not become available",
            )

            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(ModeState.MODE_TELEOP),
                timeout_s=10.0,
                failure_message="mode manager did not enter MODE_TELEOP",
            )
            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(ModeState.MODE_AI_POLICY),
                timeout_s=10.0,
                failure_message="mode manager did not enter MODE_AI_POLICY",
            )

            deadline = time.time() + 10.0
            while time.time() < deadline and not self._approved_matches(
                approved_commands,
                arbitration_states,
            ):
                executor.spin_once(timeout_sec=0.05)

            self.assertTrue(
                self._approved_matches(approved_commands, arbitration_states),
                (
                    "scripted policy candidate was not approved under SOURCE_AI_POLICY"
                    f"; approved_count={len(approved_commands)},"
                    f" arbitration_states_count={len(arbitration_states)}"
                ),
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    @staticmethod
    def _mode_request(target_mode: int) -> SetMode.Request:
        request = SetMode.Request()
        request.requested_mode = target_mode
        request.requester = "scripted_policy_launch_test"
        request.reason = "scripted_policy_e2e"
        return request

    def _call_until_accepted(
        self,
        executor,
        client,
        request_factory,
        timeout_s: float,
        failure_message: str,
    ):
        deadline = time.time() + timeout_s
        last_message = ""
        while time.time() < deadline:
            future = client.call_async(request_factory())
            inner_deadline = min(deadline, time.time() + 1.0)
            while time.time() < inner_deadline and not future.done():
                executor.spin_once(timeout_sec=0.05)
            if not future.done():
                continue
            response = future.result()
            if response.accepted:
                return response
            last_message = getattr(response, "message", "")
            executor.spin_once(timeout_sec=0.05)
        self.fail(f"{failure_message}; last response: {last_message}")

    @staticmethod
    def _approved_matches(
        approved_commands,
        arbitration_states,
    ) -> bool:
        has_approval_state = any(
            state.output_enabled
            and state.active_source == CommandArbitrationState.SOURCE_AI_POLICY
            and state.reason == "approved"
            for state in arbitration_states
        )
        has_command = any(
            abs(command.twist.linear.x - PLAN_LINEAR_X) < 1e-6
            and abs(command.twist.angular.z - PLAN_ANGULAR_Z) < 1e-6
            for command in approved_commands
        )
        return has_approval_state and has_command


@launch_testing.post_shutdown_test()
class TestScriptedPolicyLaunchShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
