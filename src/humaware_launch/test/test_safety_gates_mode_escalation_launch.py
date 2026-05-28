"""End-to-end launch test for safety gating of mode escalation.

The arbiter/MRM/E-stop tests prove that an unsafe state blocks *command
output*. This test proves the complementary contract one level up: an
unsafe state blocks *mode escalation itself*. The runtime must refuse to
enter an autonomous mode (``AUTONOMY`` or ``AI_POLICY``) while safety is
not healthy, and the refusal must lift the moment safety recovers.

By toggling the safety manager's ``estop_engaged`` parameter at runtime
the test:

* engages the E-stop and asserts ``mode/set`` requests for both
  ``MODE_AUTONOMY`` and ``MODE_AI_POLICY`` are rejected with
  ``message == "safety_state_blocks_autonomy"`` and the active mode never
  escalates,
* releases the E-stop and asserts a ``MODE_AUTONOMY`` request is then
  accepted -- proving the gate is the live safety state, not a latch.

This pins the safety <-> mode coupling: the safety state machine governs
not just what the robot may emit, but what operating mode it may assume.
"""

import os
import time
import unittest

from humaware_msgs.msg import ModeState, SafetyState
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
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


ROBOT_ID = f"mock_safety_gate_{os.getpid()}"


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


class TestSafetyGatesModeEscalation(unittest.TestCase):
    def test_estop_blocks_autonomy_escalation_until_released(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/safety/state", SafetyState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"safety_gate_launch_test_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        safety_states: list = []
        mode_states: list = []
        try:
            node.create_subscription(
                SafetyState,
                f"/{robot_id}/safety/state",
                safety_states.append,
                10,
            )
            node.create_subscription(
                ModeState,
                f"/{robot_id}/mode/state",
                mode_states.append,
                10,
            )

            mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")
            param_client = AsyncParameterClient(node, f"/{robot_id}/safety_manager")

            self.assertTrue(
                mode_client.wait_for_service(timeout_sec=10.0),
                "mode/set service did not become available",
            )
            self.assertTrue(
                param_client.wait_for_services(timeout_sec=10.0),
                "safety_manager parameter services did not become available",
            )

            # --- Phase 1: engage E-stop, wait for STATE_ESTOP to settle ----
            self._set_estop(executor, param_client, True)
            self._spin_until(
                executor,
                check=lambda: self._latest_safety(safety_states) == SafetyState.STATE_ESTOP,
                timeout_s=8.0,
                failure_message="safety did not reach STATE_ESTOP after engaging E-stop",
            )
            # Let the mode manager observe the ESTOP safety state.
            self._spin_for(executor, 0.6)

            # --- Phase 2: escalation to autonomous modes must be rejected --
            for target in (ModeState.MODE_AUTONOMY, ModeState.MODE_AI_POLICY):
                response = self._call_service(
                    executor=executor,
                    client=mode_client,
                    request=self._mode_request(target),
                    timeout_s=10.0,
                    failure_message=f"mode/set({target}) did not respond under E-stop",
                )
                self.assertFalse(
                    response.accepted,
                    f"mode/set({target}) was accepted while E-stop engaged",
                )
                self.assertEqual(
                    response.message,
                    "safety_state_blocks_autonomy",
                    f"mode/set({target}) rejected for the wrong reason:"
                    f" {response.message!r}",
                )
                self.assertNotIn(
                    response.active_mode,
                    (ModeState.MODE_AUTONOMY, ModeState.MODE_AI_POLICY),
                    "active mode escalated to an autonomous mode under E-stop",
                )

            # The mode stream must never show an autonomous mode under E-stop.
            self.assertFalse(
                [
                    s
                    for s in mode_states
                    if s.active_mode
                    in (ModeState.MODE_AUTONOMY, ModeState.MODE_AI_POLICY)
                ],
                "mode/state reported an autonomous mode while E-stop engaged",
            )

            # --- Phase 3: release E-stop, escalation now allowed -----------
            self._set_estop(executor, param_client, False)
            self._spin_until(
                executor,
                check=lambda: self._latest_safety(safety_states)
                in (SafetyState.STATE_OK, SafetyState.STATE_WARN),
                timeout_s=8.0,
                failure_message="safety did not recover after releasing E-stop",
            )

            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(ModeState.MODE_AUTONOMY),
                timeout_s=10.0,
                failure_message="AUTONOMY was not accepted after E-stop release",
            )
            self._spin_until(
                executor,
                check=lambda: self._latest_mode(mode_states) == ModeState.MODE_AUTONOMY,
                timeout_s=8.0,
                failure_message="mode/state did not reach AUTONOMY after release",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    @staticmethod
    def _latest_safety(safety_states):
        return safety_states[-1].state if safety_states else None

    @staticmethod
    def _latest_mode(mode_states):
        return mode_states[-1].active_mode if mode_states else None

    def _set_estop(self, executor, param_client, engaged: bool) -> None:
        future = param_client.set_parameters(
            [Parameter("estop_engaged", Parameter.Type.BOOL, engaged)]
        )
        deadline = time.time() + 10.0
        while time.time() < deadline and not future.done():
            executor.spin_once(timeout_sec=0.05)
        self.assertTrue(
            future.done(),
            f"set_parameters(estop_engaged={engaged}) did not complete",
        )
        results = future.result().results
        self.assertTrue(
            results and results[0].successful,
            f"set_parameters(estop_engaged={engaged}) was rejected",
        )

    def _spin_until(self, executor, check, timeout_s, failure_message) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if check():
                return
        self.fail(failure_message)

    @staticmethod
    def _spin_for(executor, duration_s: float) -> None:
        deadline = time.time() + duration_s
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.05)

    @staticmethod
    def _mode_request(target_mode: int) -> SetMode.Request:
        request = SetMode.Request()
        request.requested_mode = target_mode
        request.requester = "safety_gate_launch_test"
        request.reason = "safety_gate_e2e"
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
class TestSafetyGatesModeEscalationShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
