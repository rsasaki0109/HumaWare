"""End-to-end launch test for safety lockdown of the Nav2 autonomy path.

``test_nav2_bridge_autonomy_launch`` proves the happy path: a Nav2
planner reaches the actuators through the bridge -> arbiter chain while
in ``MODE_AUTONOMY``. This test proves the *safety* half of that
contract: when an MRM is declared mid-drive, the autonomy command must be
silenced at **both** layers.

It enters ``MODE_AUTONOMY``, streams ``nav2/cmd_vel`` until the command is
approved under ``SOURCE_AUTONOMY``, then trips the safety manager via
``safety/trigger_mrm`` while continuing to stream the planner command. It
asserts that:

* the Nav2 bridge drops to ``STATE_BLOCKED`` with
  ``reason == "safety_state_blocks_navigation"`` and ``output_enabled``
  false (the bridge stops forwarding),
* the arbiter locks down under ``SOURCE_SAFETY`` with a zero stop command
  and ``reason == "safety_state_blocks_output"``,
* the still-flowing planner velocity never leaks into
  ``cmd_vel/approved`` while the MRM is engaged.

This pins the deployment-critical contract that an MRM silences the
autonomy planner just as decisively as it silences a teleop operator.
"""

import os
import time
import unittest

from geometry_msgs.msg import Twist, TwistStamped
from humaware_msgs.msg import (
    CommandArbitrationState,
    ModeState,
    NavigationBridgeState,
    SafetyState,
)
from humaware_msgs.srv import SetMode, TriggerMRM
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


ROBOT_ID = f"mock_nav2_mrm_{os.getpid()}"
NAV2_LINEAR = 0.12
NAV2_ANGULAR = 0.04
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
            "enable_nav2_bridge": "true",
        }.items(),
    )

    return launch.LaunchDescription([bringup, launch_testing.actions.ReadyToTest()]), {
        "robot_id": ROBOT_ID,
    }


class TestNav2BridgeMrmBlocks(unittest.TestCase):
    def test_mrm_silences_nav2_autonomy_path(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/navigation/nav2_bridge_state", NavigationBridgeState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"nav2_mrm_launch_test_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        approved_commands: list = []
        arbitration_states: list = []
        bridge_states: list = []
        safety_states: list = []
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
            node.create_subscription(
                NavigationBridgeState,
                f"/{robot_id}/navigation/nav2_bridge_state",
                bridge_states.append,
                10,
            )
            node.create_subscription(
                SafetyState,
                f"/{robot_id}/safety/state",
                safety_states.append,
                10,
            )
            nav2_pub = node.create_publisher(
                Twist, f"/{robot_id}/nav2/cmd_vel", 10
            )

            mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")
            trigger_client = node.create_client(
                TriggerMRM, f"/{robot_id}/safety/trigger_mrm"
            )
            for client, name in (
                (mode_client, "mode/set"),
                (trigger_client, "safety/trigger_mrm"),
            ):
                self.assertTrue(
                    client.wait_for_service(timeout_sec=10.0),
                    f"{name} service did not become available",
                )

            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(ModeState.MODE_AUTONOMY),
                timeout_s=10.0,
                failure_message="mode manager did not enter MODE_AUTONOMY",
            )

            # --- Phase 1: Nav2 command approved under AUTONOMY --------------
            self._stream_until(
                executor=executor,
                nav2_pub=nav2_pub,
                check=lambda: self._nav2_command_approved(
                    approved_commands, arbitration_states, bridge_states
                ),
                timeout_s=10.0,
                failure_message="Nav2 command was never approved before MRM",
            )

            # --- Phase 2: trigger MRM mid-drive ----------------------------
            trigger_request = TriggerMRM.Request()
            trigger_request.requester = "nav2_mrm_launch_test"
            trigger_request.reason = "test_blocks_nav2"
            trigger_response = self._call_service(
                executor=executor,
                client=trigger_client,
                request=trigger_request,
                timeout_s=10.0,
                failure_message="trigger_mrm service did not respond",
            )
            self.assertTrue(
                trigger_response.accepted, "trigger_mrm rejected by safety_manager"
            )

            approved_commands.clear()
            arbitration_states.clear()
            bridge_states.clear()
            safety_states.clear()

            self._stream_until(
                executor=executor,
                nav2_pub=nav2_pub,
                check=lambda: self._nav2_lockdown_observed(
                    arbitration_states, bridge_states, safety_states
                ),
                timeout_s=10.0,
                failure_message="MRM lockdown of the Nav2 path was not observed",
            )

            # --- Phase 2b: hold the lockdown and assert no planner leak ----
            approved_commands.clear()
            arbitration_states.clear()
            bridge_states.clear()
            deadline = time.time() + 1.5
            while time.time() < deadline:
                nav2_pub.publish(self._twist(NAV2_LINEAR, NAV2_ANGULAR))
                executor.spin_once(timeout_sec=0.05)

            nav2_leaks = [
                c
                for c in approved_commands
                if abs(c.twist.linear.x - NAV2_LINEAR) < MATCH_TOL
                and abs(c.twist.angular.z - NAV2_ANGULAR) < MATCH_TOL
            ]
            non_safety_states = [
                s
                for s in arbitration_states
                if s.output_enabled
                or s.active_source != CommandArbitrationState.SOURCE_SAFETY
            ]
            bridge_forwarding = [
                s for s in bridge_states if s.output_enabled
            ]
            self.assertFalse(
                nav2_leaks,
                f"Nav2 command leaked into approved while MRM engaged:"
                f" {len(nav2_leaks)} sample(s)",
            )
            self.assertFalse(
                non_safety_states,
                "arbitration_state showed output_enabled or non-SAFETY source"
                " while MRM engaged",
            )
            self.assertFalse(
                bridge_forwarding,
                "Nav2 bridge kept output_enabled while MRM engaged",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    def _stream_until(
        self,
        executor,
        nav2_pub,
        check,
        timeout_s: float,
        failure_message: str,
    ) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            nav2_pub.publish(self._twist(NAV2_LINEAR, NAV2_ANGULAR))
            executor.spin_once(timeout_sec=0.05)
            if check():
                return
        self.fail(failure_message)

    @staticmethod
    def _nav2_command_approved(approved, arbitration, bridge) -> bool:
        bridge_active = any(
            s.state == NavigationBridgeState.STATE_ACTIVE and s.output_enabled
            for s in bridge
        )
        approved_match = any(
            abs(c.twist.linear.x - NAV2_LINEAR) < MATCH_TOL
            and abs(c.twist.angular.z - NAV2_ANGULAR) < MATCH_TOL
            for c in approved
        )
        arbiter_autonomy = any(
            s.output_enabled
            and s.active_source == CommandArbitrationState.SOURCE_AUTONOMY
            and s.reason == "approved"
            for s in arbitration
        )
        return bridge_active and approved_match and arbiter_autonomy

    @staticmethod
    def _nav2_lockdown_observed(arbitration, bridge, safety) -> bool:
        safety_in_mrm = any(
            s.state == SafetyState.STATE_MRM and s.mrm_active for s in safety
        )
        bridge_blocked = any(
            s.state == NavigationBridgeState.STATE_BLOCKED
            and not s.output_enabled
            and s.reason == "safety_state_blocks_navigation"
            for s in bridge
        )
        arbiter_locked = any(
            (not s.output_enabled)
            and s.active_source == CommandArbitrationState.SOURCE_SAFETY
            and s.stop_command_published
            and s.reason == "safety_state_blocks_output"
            for s in arbitration
        )
        return safety_in_mrm and bridge_blocked and arbiter_locked

    @staticmethod
    def _twist(linear_x: float, angular_z: float) -> Twist:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    @staticmethod
    def _mode_request(target_mode: int) -> SetMode.Request:
        request = SetMode.Request()
        request.requested_mode = target_mode
        request.requester = "nav2_mrm_launch_test"
        request.reason = "nav2_mrm_e2e"
        return request

    def _call_service(
        self,
        executor,
        client,
        request,
        timeout_s: float,
        failure_message: str,
    ):
        future = client.call_async(request)
        deadline = time.time() + timeout_s
        while time.time() < deadline and not future.done():
            executor.spin_once(timeout_sec=0.05)
        self.assertTrue(future.done(), failure_message)
        return future.result()

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
class TestNav2BridgeMrmBlocksShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
