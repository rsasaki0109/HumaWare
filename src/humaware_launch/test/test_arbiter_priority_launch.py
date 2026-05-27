"""End-to-end launch test for command arbiter source priority.

The test publishes candidate velocities on both ``teleop/cmd_vel`` and
``policy/cmd_vel`` concurrently, then flips the active mode and verifies
that ``cmd_vel/approved`` only ever carries the candidate whose
``required_mode`` matches the current active mode. This proves the
priority contract claimed in PLAN.md: teleop is always preferred when
the operator owns the active mode, and the AI policy lane is unable to
sneak commands through while teleop is active.
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
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


ROBOT_ID = f"mock_priority_{os.getpid()}"
TELEOP_LINEAR = 0.13
TELEOP_ANGULAR = 0.05
POLICY_LINEAR = 0.07
POLICY_ANGULAR = 0.11
MATCH_TOL = 1e-6


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


class TestArbiterPriority(unittest.TestCase):
    def test_active_mode_overrides_concurrent_candidates(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/safety/state", SafetyState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"arbiter_priority_test_{os.getpid()}",
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
            teleop_pub = node.create_publisher(
                TwistStamped, f"/{robot_id}/teleop/cmd_vel", 10
            )
            policy_pub = node.create_publisher(
                TwistStamped, f"/{robot_id}/policy/cmd_vel", 10
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

            self._assert_wins(
                executor=executor,
                approved=approved_commands,
                arbitration=arbitration_states,
                teleop_pub=teleop_pub,
                policy_pub=policy_pub,
                expected_source=CommandArbitrationState.SOURCE_TELEOP,
                expected_linear=TELEOP_LINEAR,
                expected_angular=TELEOP_ANGULAR,
                forbidden_linear=POLICY_LINEAR,
                forbidden_angular=POLICY_ANGULAR,
                phase_label="teleop_mode",
            )

            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(ModeState.MODE_AI_POLICY),
                timeout_s=10.0,
                failure_message="mode manager did not enter MODE_AI_POLICY",
            )

            self._assert_wins(
                executor=executor,
                approved=approved_commands,
                arbitration=arbitration_states,
                teleop_pub=teleop_pub,
                policy_pub=policy_pub,
                expected_source=CommandArbitrationState.SOURCE_AI_POLICY,
                expected_linear=POLICY_LINEAR,
                expected_angular=POLICY_ANGULAR,
                forbidden_linear=TELEOP_LINEAR,
                forbidden_angular=TELEOP_ANGULAR,
                phase_label="ai_policy_mode",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    def _assert_wins(
        self,
        executor,
        approved,
        arbitration,
        teleop_pub,
        policy_pub,
        expected_source: int,
        expected_linear: float,
        expected_angular: float,
        forbidden_linear: float,
        forbidden_angular: float,
        phase_label: str,
    ) -> None:
        # Let the new ModeState propagate to the arbiter and any
        # in-flight stale candidate to age out (command_timeout_s=0.5).
        deadline = time.time() + 1.0
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.05)
        approved.clear()
        arbitration.clear()

        deadline = time.time() + 3.0
        observed = False
        while time.time() < deadline and not observed:
            teleop_pub.publish(self._twist(TELEOP_LINEAR, TELEOP_ANGULAR))
            policy_pub.publish(self._twist(POLICY_LINEAR, POLICY_ANGULAR))
            executor.spin_once(timeout_sec=0.05)
            observed = self._approved_for_source(
                approved, arbitration, expected_source, expected_linear, expected_angular
            )

        expected_hits = [
            c
            for c in approved
            if abs(c.twist.linear.x - expected_linear) < MATCH_TOL
            and abs(c.twist.angular.z - expected_angular) < MATCH_TOL
        ]
        forbidden_hits = [
            c
            for c in approved
            if abs(c.twist.linear.x - forbidden_linear) < MATCH_TOL
            and abs(c.twist.angular.z - forbidden_angular) < MATCH_TOL
        ]
        source_states = [
            s
            for s in arbitration
            if s.output_enabled
            and s.active_source == expected_source
            and s.reason == "approved"
        ]

        self.assertTrue(
            expected_hits,
            (
                f"[{phase_label}] expected candidate from source={expected_source} was"
                f" never approved; approved_count={len(approved)},"
                f" arbitration_states_count={len(arbitration)}"
            ),
        )
        self.assertFalse(
            forbidden_hits,
            (
                f"[{phase_label}] forbidden candidate (linear={forbidden_linear},"
                f" angular={forbidden_angular}) leaked through arbiter:"
                f" {len(forbidden_hits)} sample(s)"
            ),
        )
        self.assertTrue(
            source_states,
            (
                f"[{phase_label}] no CommandArbitrationState with"
                f" active_source={expected_source} and reason=approved was observed"
            ),
        )

    @staticmethod
    def _approved_for_source(
        approved,
        arbitration,
        expected_source: int,
        expected_linear: float,
        expected_angular: float,
    ) -> bool:
        has_state = any(
            s.output_enabled
            and s.active_source == expected_source
            and s.reason == "approved"
            for s in arbitration
        )
        has_command = any(
            abs(c.twist.linear.x - expected_linear) < MATCH_TOL
            and abs(c.twist.angular.z - expected_angular) < MATCH_TOL
            for c in approved
        )
        return has_state and has_command

    @staticmethod
    def _twist(linear_x: float, angular_z: float) -> TwistStamped:
        msg = TwistStamped()
        msg.header.frame_id = ROBOT_ID
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg

    @staticmethod
    def _mode_request(target_mode: int) -> SetMode.Request:
        request = SetMode.Request()
        request.requested_mode = target_mode
        request.requester = "arbiter_priority_launch_test"
        request.reason = "arbiter_priority_e2e"
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


@launch_testing.post_shutdown_test()
class TestArbiterPriorityShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
