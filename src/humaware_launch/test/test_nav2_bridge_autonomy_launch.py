"""End-to-end launch test for the Nav2 bridge AUTONOMY command path.

The teleop (``test_takeover_launch``/``test_arbiter_priority_launch``)
and AI-policy (``test_scripted_policy_launch``) command sources already
have launch coverage. This test closes the third source: a Nav2-style
planner publishing ``nav2/cmd_vel`` while the runtime is in
``MODE_AUTONOMY``.

It brings up ``mock_bringup`` with the Nav2 bridge enabled, enters
``MODE_AUTONOMY``, and streams a plain ``geometry_msgs/Twist`` on
``nav2/cmd_vel`` (the Nav2-native interface). It asserts that:

* the bridge reports ``STATE_ACTIVE`` with ``output_enabled`` and
  ``reason == "forwarding_nav2_command"``, forwarding to
  ``autonomy/cmd_vel``,
* the arbiter approves the forwarded command under ``SOURCE_AUTONOMY``
  with ``reason == "approved"``,
* the approved velocity on ``cmd_vel/approved`` matches what the planner
  requested.

This pins the contract that an autonomy planner reaches the actuators
only through the bridge -> arbiter chain, and only while AUTONOMY is the
active mode.
"""

import os
import time
import unittest

from geometry_msgs.msg import Twist, TwistStamped
from humaware_msgs.msg import (
    CommandArbitrationState,
    ModeState,
    NavigationBridgeState,
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


ROBOT_ID = f"mock_nav2_{os.getpid()}"
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


class TestNav2BridgeAutonomy(unittest.TestCase):
    def test_nav2_command_reaches_arbiter_in_autonomy(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/navigation/nav2_bridge_state", NavigationBridgeState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"nav2_launch_test_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        approved_commands: list = []
        arbitration_states: list = []
        bridge_states: list = []
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
            nav2_pub = node.create_publisher(
                Twist, f"/{robot_id}/nav2/cmd_vel", 10
            )

            mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")
            self.assertTrue(
                mode_client.wait_for_service(timeout_sec=10.0),
                "mode/set service did not become available",
            )

            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(ModeState.MODE_AUTONOMY),
                timeout_s=10.0,
                failure_message="mode manager did not enter MODE_AUTONOMY",
            )

            self._stream_until(
                executor=executor,
                nav2_pub=nav2_pub,
                check=lambda: self._nav2_command_approved(
                    approved_commands, arbitration_states, bridge_states
                ),
                timeout_s=10.0,
                failure_message=(
                    "Nav2 command never reached the arbiter under AUTONOMY;"
                    f" approved={len(approved_commands)},"
                    f" arbitration={len(arbitration_states)},"
                    f" bridge={len(bridge_states)}"
                ),
            )

            active_bridge = [
                s
                for s in bridge_states
                if s.state == NavigationBridgeState.STATE_ACTIVE
            ]
            self.assertTrue(active_bridge, "bridge never reported STATE_ACTIVE")
            for status in active_bridge[:5]:
                self.assertTrue(
                    status.output_enabled,
                    "bridge STATE_ACTIVE without output_enabled",
                )
                self.assertEqual(status.reason, "forwarding_nav2_command")
                self.assertEqual(status.output_topic, "autonomy/cmd_vel")
                self.assertEqual(status.active_mode, ModeState.MODE_AUTONOMY)
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
    def _twist(linear_x: float, angular_z: float) -> Twist:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    @staticmethod
    def _mode_request(target_mode: int) -> SetMode.Request:
        request = SetMode.Request()
        request.requested_mode = target_mode
        request.requester = "nav2_launch_test"
        request.reason = "nav2_autonomy_e2e"
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
class TestNav2BridgeAutonomyShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
