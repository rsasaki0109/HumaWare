"""End-to-end launch test for the MRM (minimal risk maneuver) lockdown path.

The test enters MODE_TELEOP, streams a candidate velocity, then trips the
safety manager via ``safety/trigger_mrm``. It asserts that:

* ``safety/state.state`` transitions to ``STATE_MRM``,
* ``cmd_vel/approved`` switches to all-zero TwistStamped (stop command),
* ``CommandArbitrationState`` reports ``active_source == SOURCE_SAFETY``
  with ``output_enabled=false``, ``stop_command_published=true``, and
  ``reason == "safety_state_blocks_output"``,
* while MRM is engaged the teleop candidate never leaks back into
  ``cmd_vel/approved``,
* once ``safety/clear_mrm`` is called the teleop candidate is approved
  again under ``SOURCE_TELEOP``.

This pins the deployment-critical contract that an MRM declaration must
immediately silence the arbiter, regardless of what any actor is still
publishing on the candidate lanes.
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
from humaware_msgs.srv import ClearMRM, SetMode, TriggerMRM
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


ROBOT_ID = f"mock_mrm_{os.getpid()}"
TELEOP_LINEAR = 0.13
TELEOP_ANGULAR = 0.05
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


class TestMrmBlocksOutput(unittest.TestCase):
    def test_mrm_blocks_output_and_clear_recovers(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/safety/state", SafetyState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"mrm_launch_test_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        approved_commands: list = []
        arbitration_states: list = []
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
                SafetyState,
                f"/{robot_id}/safety/state",
                safety_states.append,
                10,
            )
            teleop_pub = node.create_publisher(
                TwistStamped, f"/{robot_id}/teleop/cmd_vel", 10
            )

            mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")
            trigger_client = node.create_client(
                TriggerMRM, f"/{robot_id}/safety/trigger_mrm"
            )
            clear_client = node.create_client(
                ClearMRM, f"/{robot_id}/safety/clear_mrm"
            )
            for client, name in (
                (mode_client, "mode/set"),
                (trigger_client, "safety/trigger_mrm"),
                (clear_client, "safety/clear_mrm"),
            ):
                self.assertTrue(
                    client.wait_for_service(timeout_sec=10.0),
                    f"{name} service did not become available",
                )

            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(ModeState.MODE_TELEOP),
                timeout_s=10.0,
                failure_message="mode manager did not enter MODE_TELEOP",
            )

            # --- Phase 1: pre-MRM teleop is approved ------------------------
            self._stream_until(
                executor=executor,
                teleop_pub=teleop_pub,
                check=lambda: self._teleop_approved_recently(
                    approved_commands, arbitration_states
                ),
                timeout_s=8.0,
                failure_message="teleop candidate was never approved before MRM",
            )

            # --- Phase 2: trigger MRM ---------------------------------------
            trigger_request = TriggerMRM.Request()
            trigger_request.requester = "mrm_launch_test"
            trigger_request.reason = "test_blocks_output"
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
            safety_states.clear()

            self._stream_until(
                executor=executor,
                teleop_pub=teleop_pub,
                check=lambda: self._mrm_lockdown_observed(
                    approved_commands, arbitration_states, safety_states
                ),
                timeout_s=8.0,
                failure_message="MRM lockdown was not observed end-to-end",
            )

            # --- Phase 2b: hold the lockdown for a window and assert that
            # the teleop candidate stays blocked across multiple samples.
            approved_commands.clear()
            arbitration_states.clear()
            deadline = time.time() + 1.5
            while time.time() < deadline:
                teleop_pub.publish(self._twist(TELEOP_LINEAR, TELEOP_ANGULAR))
                executor.spin_once(timeout_sec=0.05)

            teleop_leaks = [
                c
                for c in approved_commands
                if abs(c.twist.linear.x - TELEOP_LINEAR) < MATCH_TOL
                and abs(c.twist.angular.z - TELEOP_ANGULAR) < MATCH_TOL
            ]
            non_safety_states = [
                s
                for s in arbitration_states
                if s.output_enabled
                or s.active_source != CommandArbitrationState.SOURCE_SAFETY
            ]
            self.assertFalse(
                teleop_leaks,
                f"teleop candidate leaked into approved while MRM engaged:"
                f" {len(teleop_leaks)} sample(s)",
            )
            self.assertFalse(
                non_safety_states,
                "arbitration_state showed output_enabled=true or non-SAFETY source"
                " while MRM engaged",
            )

            # --- Phase 3: clear MRM -----------------------------------------
            clear_request = ClearMRM.Request()
            clear_request.requester = "mrm_launch_test"
            clear_request.reason = "test_recovery"
            clear_response = self._call_service(
                executor=executor,
                client=clear_client,
                request=clear_request,
                timeout_s=10.0,
                failure_message="clear_mrm service did not respond",
            )
            self.assertTrue(
                clear_response.accepted, "clear_mrm rejected by safety_manager"
            )

            approved_commands.clear()
            arbitration_states.clear()
            safety_states.clear()

            self._stream_until(
                executor=executor,
                teleop_pub=teleop_pub,
                check=lambda: (
                    self._teleop_approved_recently(approved_commands, arbitration_states)
                    and any(
                        s.state != SafetyState.STATE_MRM and not s.mrm_active
                        for s in safety_states
                    )
                ),
                timeout_s=8.0,
                failure_message="runtime did not recover after clear_mrm",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    def _stream_until(
        self,
        executor,
        teleop_pub,
        check,
        timeout_s: float,
        failure_message: str,
    ) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            teleop_pub.publish(self._twist(TELEOP_LINEAR, TELEOP_ANGULAR))
            executor.spin_once(timeout_sec=0.05)
            if check():
                return
        self.fail(failure_message)

    @staticmethod
    def _teleop_approved_recently(approved, arbitration) -> bool:
        has_command = any(
            abs(c.twist.linear.x - TELEOP_LINEAR) < MATCH_TOL
            and abs(c.twist.angular.z - TELEOP_ANGULAR) < MATCH_TOL
            for c in approved
        )
        has_state = any(
            s.output_enabled
            and s.active_source == CommandArbitrationState.SOURCE_TELEOP
            and s.reason == "approved"
            for s in arbitration
        )
        return has_command and has_state

    @staticmethod
    def _mrm_lockdown_observed(approved, arbitration, safety) -> bool:
        safety_in_mrm = any(
            s.state == SafetyState.STATE_MRM and s.mrm_active for s in safety
        )
        arbiter_locked = any(
            (not s.output_enabled)
            and s.active_source == CommandArbitrationState.SOURCE_SAFETY
            and s.stop_command_published
            and s.reason == "safety_state_blocks_output"
            for s in arbitration
        )
        stop_published = any(
            abs(c.twist.linear.x) < MATCH_TOL
            and abs(c.twist.linear.y) < MATCH_TOL
            and abs(c.twist.angular.z) < MATCH_TOL
            for c in approved
        )
        return safety_in_mrm and arbiter_locked and stop_published

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
        request.requester = "mrm_launch_test"
        request.reason = "mrm_e2e"
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


@launch_testing.post_shutdown_test()
class TestMrmBlocksOutputShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
