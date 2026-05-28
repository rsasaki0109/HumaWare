"""End-to-end launch test for the approved-command watchdog auto-MRM path.

This is the third leg of the safety manager's watchdog matrix, after the
teleop heartbeat (``test_heartbeat_timeout_mrm_launch``) and hardware
heartbeat (``test_hardware_heartbeat_timeout_mrm_launch``) watchdogs. It
covers the ``approved_command_timeout`` watchdog: the safety manager
escalates to an MRM when the *approved* command stream stalls while the
runtime is in an active mode.

To exercise this path the arbiter is launched with
``arbiter_publish_stop_on_block:=false`` so that, when the teleop
candidate goes stale, the arbiter stops emitting commands on
``cmd_vel/approved`` entirely (rather than flooding it with zero stops).
That configuration -- stop authority delegated to the safety manager --
is exactly when the approved-command watchdog is the active backstop.

The teleop *heartbeat* is streamed throughout both phases so the teleop
heartbeat watchdog never fires; the stalled approved-command stream is
the sole trigger. The test:

* enters ``MODE_TELEOP`` and streams ``teleop/heartbeat`` +
  ``teleop/cmd_vel`` until teleop is approved while safety is not in MRM,
* stops ``teleop/cmd_vel`` (keeping the heartbeat), so the approved
  stream dries up,
* asserts ``safety/state`` transitions to ``STATE_MRM`` with
  ``approved_command_timeout`` in its warnings and ``mrm_state`` reports
  ``STATE_STOP``.
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
from std_msgs.msg import Header


ROBOT_ID = f"mock_approved_mrm_{os.getpid()}"
TELEOP_LINEAR = 0.10
TELEOP_ANGULAR = 0.05
MATCH_TOL = 1e-6
APPROVED_TIMEOUT_S = 0.5


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
            "approved_command_timeout_s": str(APPROVED_TIMEOUT_S),
            "approved_command_timeout_triggers_mrm": "true",
            "arbiter_publish_stop_on_block": "false",
        }.items(),
    )

    return launch.LaunchDescription([bringup, launch_testing.actions.ReadyToTest()]), {
        "robot_id": ROBOT_ID,
    }


class TestApprovedCommandTimeoutMrm(unittest.TestCase):
    def test_approved_command_stall_auto_triggers_mrm(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/safety/state", SafetyState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"approved_mrm_launch_test_{os.getpid()}",
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

            # --- Phase 1: heartbeat + command flowing -> teleop approved,
            #              safety not in MRM. -------------------------------
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

            # --- Phase 2: command stops, heartbeat keeps flowing. ----------
            approved_commands.clear()
            arbitration_states.clear()
            safety_states.clear()
            mrm_states.clear()

            self._stream_until(
                executor=executor,
                publish=lambda: teleop_hb_pub.publish(self._header()),
                check=lambda: self._approved_timeout_mrm_observed(
                    safety_states, mrm_states
                ),
                timeout_s=8.0,
                failure_message=(
                    "approved-command stall did not auto-trigger an MRM lockdown"
                ),
            )

            # --- Phase 2b: hold the stalled window and assert no teleop
            # command leaks back into approved (none is being published). ---
            approved_commands.clear()
            deadline = time.time() + 1.5
            while time.time() < deadline:
                teleop_hb_pub.publish(self._header())
                executor.spin_once(timeout_sec=0.05)

            teleop_leaks = [
                c
                for c in approved_commands
                if abs(c.twist.linear.x - TELEOP_LINEAR) < MATCH_TOL
                and abs(c.twist.angular.z - TELEOP_ANGULAR) < MATCH_TOL
            ]
            self.assertFalse(
                teleop_leaks,
                "teleop command leaked into approved after approved-command MRM:"
                f" {len(teleop_leaks)} sample(s)",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    def _publish_heartbeat_and_command(self, hb_pub, cmd_pub) -> None:
        hb_pub.publish(self._header())
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
            s.state != SafetyState.STATE_MRM and not s.mrm_active for s in safety
        )
        return has_command and has_state and safety_healthy

    @staticmethod
    def _approved_timeout_mrm_observed(safety, mrm) -> bool:
        safety_in_mrm = any(
            s.state == SafetyState.STATE_MRM
            and s.mrm_active
            and "approved_command_timeout" in s.active_warnings
            for s in safety
        )
        mrm_stopped = any(m.state == MRMState.STATE_STOP for m in mrm)
        return safety_in_mrm and mrm_stopped

    @staticmethod
    def _header() -> Header:
        header = Header()
        header.frame_id = ROBOT_ID
        return header

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
        request.requester = "approved_mrm_launch_test"
        request.reason = "approved_command_mrm_e2e"
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
class TestApprovedCommandTimeoutMrmShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
