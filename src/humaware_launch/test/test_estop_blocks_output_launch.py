"""End-to-end launch test for the E-stop lockdown path.

E-stop sits above MRM in the safety state machine: it is the terminal
hardware-authority stop. This test drives the runtime through the full
E-stop lifecycle by toggling the safety manager's ``estop_engaged``
parameter at runtime (the deployment stand-in for a physical E-stop
line):

* enters ``MODE_TELEOP`` and streams ``teleop/heartbeat`` +
  ``teleop/cmd_vel`` until teleop is approved while safety is healthy,
* engages the E-stop and asserts ``safety/state`` becomes ``STATE_ESTOP``
  with ``estop_engaged`` set and ``estop_engaged`` in ``active_faults``,
  ``mrm_state`` reports ``STATE_FAULT``, and the arbiter locks down under
  ``SOURCE_SAFETY`` with a zero stop and ``safety_state_blocks_output``,
* asserts a ``clear_mrm`` request is **rejected** while the E-stop is
  engaged (E-stop outranks the MRM clear path),
* disengages the E-stop and asserts the runtime recovers: teleop is
  approved again and safety leaves ``STATE_ESTOP``.

This pins the contract that an E-stop silences every command source, can
only be released by clearing the E-stop itself (not by clearing the MRM),
and that release restores normal operation.
"""

import os
import time
import unittest

from geometry_msgs.msg import TwistStamped
from humaware_msgs.msg import (
    CommandArbitrationState,
    ModeState,
    MRMState,
    SafetyState,
)
from humaware_msgs.srv import ClearMRM, SetMode
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
from std_msgs.msg import Header


ROBOT_ID = f"mock_estop_{os.getpid()}"
TELEOP_LINEAR = 0.11
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


class TestEstopBlocksOutput(unittest.TestCase):
    def test_estop_blocks_output_and_release_recovers(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/safety/state", SafetyState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"estop_launch_test_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        approved_commands: list = []
        arbitration_states: list = []
        safety_states: list = []
        mrm_states: list = []
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
            node.create_subscription(
                MRMState,
                f"/{robot_id}/safety/mrm_state",
                mrm_states.append,
                10,
            )
            teleop_cmd_pub = node.create_publisher(
                TwistStamped, f"/{robot_id}/teleop/cmd_vel", 10
            )
            teleop_hb_pub = node.create_publisher(
                Header, f"/{robot_id}/teleop/heartbeat", 10
            )

            mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")
            clear_client = node.create_client(ClearMRM, f"/{robot_id}/safety/clear_mrm")
            param_client = AsyncParameterClient(node, f"/{robot_id}/safety_manager")

            self.assertTrue(
                mode_client.wait_for_service(timeout_sec=10.0),
                "mode/set service did not become available",
            )
            self.assertTrue(
                clear_client.wait_for_service(timeout_sec=10.0),
                "safety/clear_mrm service did not become available",
            )
            self.assertTrue(
                param_client.wait_for_services(timeout_sec=10.0),
                "safety_manager parameter services did not become available",
            )

            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(ModeState.MODE_TELEOP),
                timeout_s=10.0,
                failure_message="mode manager did not enter MODE_TELEOP",
            )

            # --- Phase 1: teleop approved, safety healthy ------------------
            self._stream_until(
                executor=executor,
                publish=lambda: self._publish_heartbeat_and_command(
                    teleop_hb_pub, teleop_cmd_pub
                ),
                check=lambda: self._teleop_approved_and_safe(
                    approved_commands, arbitration_states, safety_states
                ),
                timeout_s=8.0,
                failure_message="teleop was never approved with safety healthy",
            )

            # --- Phase 2: engage E-stop ------------------------------------
            self._set_estop(executor, param_client, True)

            approved_commands.clear()
            arbitration_states.clear()
            safety_states.clear()
            mrm_states.clear()

            self._stream_until(
                executor=executor,
                publish=lambda: self._publish_heartbeat_and_command(
                    teleop_hb_pub, teleop_cmd_pub
                ),
                check=lambda: self._estop_lockdown_observed(
                    arbitration_states, safety_states, mrm_states
                ),
                timeout_s=8.0,
                failure_message="E-stop lockdown was not observed end-to-end",
            )

            # --- Phase 2b: clear_mrm must be rejected under E-stop ---------
            clear_request = ClearMRM.Request()
            clear_request.requester = "estop_launch_test"
            clear_request.reason = "should_be_rejected"
            clear_response = self._call_service(
                executor=executor,
                client=clear_client,
                request=clear_request,
                timeout_s=10.0,
                failure_message="clear_mrm service did not respond",
            )
            self.assertFalse(
                clear_response.accepted,
                "clear_mrm was accepted while E-stop engaged",
            )
            self.assertEqual(
                clear_response.active_safety_state,
                SafetyState.STATE_ESTOP,
                "clear_mrm rejection did not report STATE_ESTOP",
            )

            # --- Phase 2c: hold the lockdown, assert no teleop leak --------
            approved_commands.clear()
            arbitration_states.clear()
            deadline = time.time() + 1.5
            while time.time() < deadline:
                self._publish_heartbeat_and_command(teleop_hb_pub, teleop_cmd_pub)
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
                f"teleop command leaked into approved while E-stop engaged:"
                f" {len(teleop_leaks)} sample(s)",
            )
            self.assertFalse(
                non_safety_states,
                "arbitration_state showed output_enabled or non-SAFETY source"
                " while E-stop engaged",
            )

            # --- Phase 3: release E-stop -> recovery -----------------------
            self._set_estop(executor, param_client, False)

            approved_commands.clear()
            arbitration_states.clear()
            safety_states.clear()

            self._stream_until(
                executor=executor,
                publish=lambda: self._publish_heartbeat_and_command(
                    teleop_hb_pub, teleop_cmd_pub
                ),
                check=lambda: (
                    self._teleop_approved_and_safe(
                        approved_commands, arbitration_states, safety_states
                    )
                    and any(s.state != SafetyState.STATE_ESTOP for s in safety_states)
                ),
                timeout_s=8.0,
                failure_message="runtime did not recover after E-stop release",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

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

    def _publish_heartbeat_and_command(self, hb_pub, cmd_pub) -> None:
        header = Header()
        header.frame_id = ROBOT_ID
        hb_pub.publish(header)
        cmd_pub.publish(self._twist(TELEOP_LINEAR, TELEOP_ANGULAR))

    def _stream_until(
        self,
        executor,
        publish,
        check,
        timeout_s: float,
        failure_message: str,
    ) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            publish()
            executor.spin_once(timeout_sec=0.05)
            if check():
                return
        self.fail(failure_message)

    @staticmethod
    def _teleop_approved_and_safe(approved, arbitration, safety) -> bool:
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
        safety_healthy = any(
            s.state in (SafetyState.STATE_OK, SafetyState.STATE_WARN)
            and not s.estop_engaged
            for s in safety
        )
        return has_command and has_state and safety_healthy

    @staticmethod
    def _estop_lockdown_observed(arbitration, safety, mrm) -> bool:
        safety_in_estop = any(
            s.state == SafetyState.STATE_ESTOP
            and s.estop_engaged
            and "estop_engaged" in s.active_faults
            for s in safety
        )
        mrm_fault = any(m.state == MRMState.STATE_FAULT for m in mrm)
        arbiter_locked = any(
            (not s.output_enabled)
            and s.active_source == CommandArbitrationState.SOURCE_SAFETY
            and s.stop_command_published
            and s.reason == "safety_state_blocks_output"
            for s in arbitration
        )
        return safety_in_estop and mrm_fault and arbiter_locked

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
        request.requester = "estop_launch_test"
        request.reason = "estop_e2e"
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
class TestEstopBlocksOutputShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
