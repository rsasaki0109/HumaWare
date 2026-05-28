"""End-to-end launch test for the hardware heartbeat watchdog auto-MRM path.

This is the hardware-side companion to
``test_heartbeat_timeout_mrm_launch`` (which covers the *teleop*
heartbeat). It proves that when the adapter's ``hardware/heartbeat``
stops arriving while the runtime is in an active mode, the safety manager
declares an MRM on its own, with no human or service intervention -- the
robot must not keep accepting commands once the link to the actuators is
unverified.

The runtime is brought up with ``require_hardware_heartbeat:=true``,
``hardware_heartbeat_timeout_triggers_mrm:=true`` and a short timeout. The
test then:

* enters ``MODE_TELEOP`` and streams ``hardware/heartbeat``,
  ``teleop/heartbeat`` and ``teleop/cmd_vel`` until the teleop candidate
  is approved while safety reports a non-MRM state (both watchdogs
  satisfied),
* stops *only* the hardware heartbeat while continuing to stream the
  teleop heartbeat and command (so the teleop and approved-command
  watchdogs stay satisfied and the hardware path is the only thing that
  can trip the MRM),
* asserts that ``safety/state`` transitions to ``STATE_MRM`` with
  ``hardware_heartbeat_timeout`` in its warnings, ``mrm_state`` reports
  ``STATE_STOP``, and the arbiter locks down under ``SOURCE_SAFETY`` with
  a zero stop command,
* holds the dead-heartbeat window and asserts the still-flowing teleop
  command never leaks back into ``cmd_vel/approved``.
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


ROBOT_ID = f"mock_hw_hb_mrm_{os.getpid()}"
TELEOP_LINEAR = 0.08
TELEOP_ANGULAR = 0.06
MATCH_TOL = 1e-6
HEARTBEAT_TIMEOUT_S = 0.5


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
            "require_hardware_heartbeat": "true",
            "hardware_heartbeat_timeout_s": str(HEARTBEAT_TIMEOUT_S),
            "hardware_heartbeat_timeout_triggers_mrm": "true",
        }.items(),
    )

    return launch.LaunchDescription([bringup, launch_testing.actions.ReadyToTest()]), {
        "robot_id": ROBOT_ID,
    }


class TestHardwareHeartbeatTimeoutMrm(unittest.TestCase):
    def test_hardware_heartbeat_loss_auto_triggers_mrm(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/safety/state", SafetyState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"hw_hb_mrm_launch_test_{os.getpid()}",
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
            hardware_hb_pub = node.create_publisher(
                Header, f"/{robot_id}/hardware/heartbeat", 10
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

            # --- Phase 1: all heartbeats + command flowing -> teleop
            #              approved, safety not in MRM. ---------------------
            self._stream_until(
                executor=executor,
                publish=lambda: self._publish_all(
                    teleop_hb_pub, hardware_hb_pub, teleop_cmd_pub
                ),
                check=lambda: self._teleop_approved_and_safe(
                    approved_commands, arbitration_states, safety_states
                ),
                timeout_s=8.0,
                failure_message="teleop was never approved with safety healthy",
            )

            # --- Phase 2: hardware heartbeat stops; teleop heartbeat and
            #              command keep flowing. ------------------------------
            approved_commands.clear()
            arbitration_states.clear()
            safety_states.clear()
            mrm_states.clear()

            self._stream_until(
                executor=executor,
                publish=lambda: self._publish_teleop_only(
                    teleop_hb_pub, teleop_cmd_pub
                ),
                check=lambda: self._hardware_mrm_observed(
                    approved_commands, arbitration_states, safety_states, mrm_states
                ),
                timeout_s=8.0,
                failure_message=(
                    "hardware heartbeat loss did not auto-trigger an MRM lockdown"
                ),
            )

            # --- Phase 2b: hold the dead-heartbeat window and assert the
            # still-flowing teleop command never leaks into approved. -------
            approved_commands.clear()
            arbitration_states.clear()
            deadline = time.time() + 1.5
            while time.time() < deadline:
                self._publish_teleop_only(teleop_hb_pub, teleop_cmd_pub)
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
                "teleop command leaked into approved after hardware-heartbeat MRM:"
                f" {len(teleop_leaks)} sample(s)",
            )
            self.assertFalse(
                non_safety_states,
                "arbitration_state showed output_enabled or non-SAFETY source"
                " after hardware-heartbeat MRM",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    def _publish_all(self, teleop_hb_pub, hardware_hb_pub, cmd_pub) -> None:
        teleop_hb_pub.publish(self._header())
        hardware_hb_pub.publish(self._header())
        cmd_pub.publish(self._twist(TELEOP_LINEAR, TELEOP_ANGULAR))

    def _publish_teleop_only(self, teleop_hb_pub, cmd_pub) -> None:
        teleop_hb_pub.publish(self._header())
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
    def _hardware_mrm_observed(approved, arbitration, safety, mrm) -> bool:
        safety_in_mrm = any(
            s.state == SafetyState.STATE_MRM
            and s.mrm_active
            and "hardware_heartbeat_timeout" in s.active_warnings
            for s in safety
        )
        mrm_stopped = any(m.state == MRMState.STATE_STOP for m in mrm)
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
        return safety_in_mrm and mrm_stopped and arbiter_locked and stop_published

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
        request.requester = "hw_hb_mrm_launch_test"
        request.reason = "hardware_heartbeat_mrm_e2e"
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
class TestHardwareHeartbeatTimeoutMrmShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
