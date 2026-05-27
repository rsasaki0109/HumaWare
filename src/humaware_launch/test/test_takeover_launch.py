"""End-to-end launch tests for operator takeover.

These tests walk the runtime into AUTONOMY (then AI_POLICY) and verify
that ``mode/takeover`` flips ``ModeState.active_mode`` to ``TELEOP`` and
that ``ModeTransitionState`` carries ``takeover=True``.
"""

import os
import time
import unittest

from humaware_msgs.msg import ModeState, ModeTransitionState, SafetyState
from humaware_msgs.srv import SetMode, Takeover
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


ROBOT_ID = f"mock_takeover_{os.getpid()}"


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


class TestTakeover(unittest.TestCase):
    def test_takeover_from_autonomy_returns_to_teleop(self, robot_id):
        self._run_takeover_test(
            robot_id=robot_id,
            modes_to_enter=(ModeState.MODE_TELEOP, ModeState.MODE_AUTONOMY),
            expected_previous_mode=ModeState.MODE_AUTONOMY,
        )

    def test_takeover_from_ai_policy_returns_to_teleop(self, robot_id):
        self._run_takeover_test(
            robot_id=robot_id,
            modes_to_enter=(ModeState.MODE_TELEOP, ModeState.MODE_AI_POLICY),
            expected_previous_mode=ModeState.MODE_AI_POLICY,
        )

    def _run_takeover_test(
        self,
        robot_id: str,
        modes_to_enter: tuple,
        expected_previous_mode: int,
    ) -> None:
        with WaitForTopics(
            [(f"/{robot_id}/safety/state", SafetyState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"takeover_test_{expected_previous_mode}_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        mode_states: list = []
        transition_states: list = []
        try:
            node.create_subscription(
                ModeState,
                f"/{robot_id}/mode/state",
                mode_states.append,
                10,
            )
            node.create_subscription(
                ModeTransitionState,
                f"/{robot_id}/mode/transition_state",
                transition_states.append,
                10,
            )

            set_mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")
            takeover_client = node.create_client(Takeover, f"/{robot_id}/mode/takeover")
            self.assertTrue(
                set_mode_client.wait_for_service(timeout_sec=10.0),
                "mode/set service did not become available",
            )
            self.assertTrue(
                takeover_client.wait_for_service(timeout_sec=10.0),
                "mode/takeover service did not become available",
            )

            for target_mode in modes_to_enter:
                self._call_until_accepted(
                    executor=executor,
                    client=set_mode_client,
                    request_factory=lambda mode=target_mode: self._mode_request(mode),
                    timeout_s=10.0,
                    failure_message=f"mode manager did not enter mode {target_mode}",
                )

            self._wait_for_active_mode(
                executor=executor,
                mode_states=mode_states,
                expected_mode=expected_previous_mode,
                timeout_s=5.0,
            )

            transitions_before_takeover = len(transition_states)
            response = self._call_takeover(
                executor=executor,
                client=takeover_client,
                timeout_s=10.0,
            )
            self.assertTrue(response.accepted)
            self.assertEqual(
                response.previous_mode,
                expected_previous_mode,
                f"expected previous_mode={expected_previous_mode}, got {response.previous_mode}",
            )
            self.assertEqual(response.active_mode, ModeState.MODE_TELEOP)

            self._wait_for_active_mode(
                executor=executor,
                mode_states=mode_states,
                expected_mode=ModeState.MODE_TELEOP,
                timeout_s=5.0,
            )

            takeover_transitions = [
                event
                for event in transition_states[transitions_before_takeover:]
                if event.takeover
            ]
            self.assertTrue(
                takeover_transitions,
                "no ModeTransitionState with takeover=True was published",
            )
            self.assertEqual(takeover_transitions[-1].active_mode, ModeState.MODE_TELEOP)
            self.assertEqual(
                takeover_transitions[-1].outcome,
                ModeTransitionState.OUTCOME_ACCEPTED,
                "takeover ModeTransitionState should report OUTCOME_ACCEPTED",
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
        request.requester = "takeover_launch_test"
        request.reason = "takeover_e2e_setup"
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

    def _call_takeover(self, executor, client, timeout_s: float) -> Takeover.Response:
        request = Takeover.Request()
        request.requester = "takeover_launch_test"
        request.reason = "takeover_e2e"
        future = client.call_async(request)
        deadline = time.time() + timeout_s
        while time.time() < deadline and not future.done():
            executor.spin_once(timeout_sec=0.05)
        self.assertTrue(future.done(), "takeover service call did not complete")
        return future.result()

    def _wait_for_active_mode(
        self,
        executor,
        mode_states,
        expected_mode: int,
        timeout_s: float,
    ) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if any(state.active_mode == expected_mode for state in mode_states):
                return
            executor.spin_once(timeout_sec=0.05)
        observed = [state.active_mode for state in mode_states[-5:]]
        self.fail(
            f"never observed ModeState.active_mode={expected_mode};"
            f" last observed (up to 5): {observed}"
        )


@launch_testing.post_shutdown_test()
class TestTakeoverShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
