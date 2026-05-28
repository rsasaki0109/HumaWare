"""End-to-end launch test for the core motion + command-staleness contract.

Every other launch test proves a *blocking* contract: an unsafe state, a
lost heartbeat or an E-stop must silence the robot. This one pins the
load-bearing *positive* path and its fail-safe tail, which nothing else
exercises end to end:

* a fresh teleop command in ``MODE_TELEOP`` is approved by the arbiter
  (``reason == "approved"``, ``active_source == SOURCE_TELEOP``) and drives
  the locomotion adapter all the way to ``STATE_WALKING`` -- the runtime
  actually moves the robot when it should, and the approved velocity is the
  commanded one (within the clamp);
* when the teleop stream stops, the arbiter must *not* keep forwarding the
  last command: it ages the candidate out (``no_fresh_command_for_active_mode``),
  publishes an explicit stop (``stop_command_published``), and the locomotion
  adapter returns to ``STATE_STANDING``.

``cmd_vel/approved`` is the only topic that physically commands motion, and
the arbiter is its only producer, so this is the single most important
runtime contract: it moves on a fresh command and stops itself the instant
the command goes stale.
"""

import os
import time
import unittest

from geometry_msgs.msg import TwistStamped
from humaware_msgs.msg import (
    CommandArbitrationState,
    LocomotionState,
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


ROBOT_ID = f"mock_drive_{os.getpid()}"

TELEOP_LINEAR = 0.2
TELEOP_ANGULAR = 0.0
MATCH_TOL = 0.05


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


class TestTeleopDriveThenStaleStop(unittest.TestCase):
    def test_fresh_command_drives_then_stale_command_stops(self, robot_id):
        with WaitForTopics(
            [
                (f"/{robot_id}/safety/state", SafetyState),
                (f"/{robot_id}/locomotion/state", LocomotionState),
            ],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"drive_launch_test_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        locomotion_states: list = []
        arbitration_states: list = []
        approved_cmds: list = []
        try:
            node.create_subscription(
                LocomotionState,
                f"/{robot_id}/locomotion/state",
                locomotion_states.append,
                10,
            )
            node.create_subscription(
                CommandArbitrationState,
                f"/{robot_id}/runtime/command_arbitration_state",
                arbitration_states.append,
                10,
            )
            node.create_subscription(
                TwistStamped,
                f"/{robot_id}/cmd_vel/approved",
                approved_cmds.append,
                10,
            )
            teleop_pub = node.create_publisher(
                TwistStamped, f"/{robot_id}/teleop/cmd_vel", 10
            )
            mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")

            self.assertTrue(
                mode_client.wait_for_service(timeout_sec=10.0),
                "mode/set service did not become available",
            )

            # Escalate to TELEOP (allowed under OK/WARN once safety is seen).
            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(ModeState.MODE_TELEOP),
                timeout_s=10.0,
                failure_message="TELEOP was not accepted",
            )

            # --- Phase 1: a fresh teleop command drives the robot ----------
            arbitration_states.clear()
            approved_cmds.clear()
            self._drive_until(
                executor=executor,
                teleop_pub=teleop_pub,
                check=lambda: self._latest_locomotion(locomotion_states)
                == LocomotionState.STATE_WALKING,
                timeout_s=8.0,
                failure_message="locomotion never reached STATE_WALKING under a fresh teleop command",
            )

            approved_teleop = [
                s
                for s in arbitration_states
                if s.output_enabled
                and s.active_source == CommandArbitrationState.SOURCE_TELEOP
                and s.reason == "approved"
            ]
            self.assertTrue(
                approved_teleop,
                "arbiter never reported an approved TELEOP command",
            )
            matched_velocity = [
                c
                for c in approved_cmds
                if abs(c.twist.linear.x - TELEOP_LINEAR) < MATCH_TOL
                and abs(c.twist.angular.z - TELEOP_ANGULAR) < MATCH_TOL
            ]
            self.assertTrue(
                matched_velocity,
                "approved velocity never matched the commanded teleop velocity",
            )

            # --- Phase 2: stop commanding; the arbiter must stop itself ----
            arbitration_states.clear()
            approved_cmds.clear()
            self._spin_until(
                executor,
                check=lambda: self._has_stale_stop(arbitration_states)
                and self._latest_locomotion(locomotion_states)
                == LocomotionState.STATE_STANDING,
                timeout_s=8.0,
                failure_message=(
                    "after the teleop stream stopped, the arbiter did not publish a"
                    " stale-command stop and/or locomotion did not return to STANDING"
                ),
            )

            stale_stops = [
                s
                for s in arbitration_states
                if s.stop_command_published
                and s.reason == "no_fresh_command_for_active_mode"
            ]
            self.assertTrue(
                stale_stops,
                "no stale-command stop (no_fresh_command_for_active_mode) was published",
            )
            self.assertTrue(
                all(
                    s.active_source == CommandArbitrationState.SOURCE_SAFETY
                    for s in stale_stops
                ),
                "stale-command stop was not attributed to SOURCE_SAFETY",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    @staticmethod
    def _latest_locomotion(locomotion_states):
        return locomotion_states[-1].state if locomotion_states else None

    @staticmethod
    def _has_stale_stop(arbitration_states) -> bool:
        return any(
            s.stop_command_published
            and s.reason == "no_fresh_command_for_active_mode"
            for s in arbitration_states
        )

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
        request.requester = "drive_launch_test"
        request.reason = "drive_e2e"
        return request

    def _drive_until(self, executor, teleop_pub, check, timeout_s, failure_message) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            teleop_pub.publish(self._twist(TELEOP_LINEAR, TELEOP_ANGULAR))
            executor.spin_once(timeout_sec=0.05)
            if check():
                return
        self.fail(failure_message)

    def _spin_until(self, executor, check, timeout_s, failure_message) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if check():
                return
        self.fail(failure_message)

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
class TestTeleopDriveThenStaleStopShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
