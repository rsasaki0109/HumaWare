"""End-to-end launch test for the hardware adapter template.

Brings up ``mock_bringup`` plus the template hardware adapter, walks the
runtime into ``MODE_TELEOP``, and verifies that the template:

* publishes ``hardware/heartbeat`` while running,
* surfaces every required identity metadata key
  (``robot_model``/``firmware_version``/``sdk_version``/``git_sha``/
  ``launch_profile``) on its ``/diagnostics`` status,
* reports level OK with the open-gate reason while approved commands are
  flowing,
* flips to WARN with a blocking reason once ``safety/trigger_mrm`` is
  invoked.

This is the deployment-side proof that the template hands a real
hardware author a working runtime gate, not just unit-testable helper
functions.
"""

import os
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from humaware_msgs.msg import ModeState, SafetyState
from humaware_msgs.srv import SetMode, TriggerMRM
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Header


ROBOT_ID = f"mock_adapter_template_{os.getpid()}"
ADAPTER_NAME = f"{ROBOT_ID}/hardware_adapter_template"
TELEOP_LINEAR = 0.11
TELEOP_ANGULAR = 0.03

IDENTITY = {
    "robot_model": "template_robot",
    "firmware_version": "fw-test-0.0.1",
    "sdk_version": "sdk-test-0.0.2",
    "git_sha": "0123456",
    "launch_profile": "ci_template_launch",
}
REQUIRED_IDENTITY_KEYS = tuple(IDENTITY.keys())


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

    adapter = Node(
        package="humaware_hardware_adapter_template",
        executable="hardware_adapter_template_node",
        namespace=ROBOT_ID,
        name="hardware_adapter_template",
        output="screen",
        parameters=[
            {"robot_id": ROBOT_ID, "publish_rate_hz": 20.0, "heartbeat_rate_hz": 20.0, **IDENTITY}
        ],
    )

    return launch.LaunchDescription(
        [bringup, adapter, launch_testing.actions.ReadyToTest()]
    ), {"robot_id": ROBOT_ID}


class TestHardwareAdapterTemplate(unittest.TestCase):
    def test_template_adapter_gate_obeys_runtime_state(self, robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/safety/state", SafetyState)],
            timeout=15.0,
            messages_received_buffer_length=1,
        ):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            f"adapter_template_launch_test_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        heartbeats: list = []
        adapter_diagnostics: list = []

        def on_diagnostics(msg: DiagnosticArray) -> None:
            for status in msg.status:
                if status.name == ADAPTER_NAME:
                    adapter_diagnostics.append(status)

        try:
            node.create_subscription(
                Header,
                f"/{robot_id}/hardware/heartbeat",
                heartbeats.append,
                10,
            )
            node.create_subscription(
                DiagnosticArray,
                "/diagnostics",
                on_diagnostics,
                10,
            )
            teleop_pub = node.create_publisher(
                TwistStamped, f"/{robot_id}/teleop/cmd_vel", 10
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
                request_factory=lambda: self._mode_request(ModeState.MODE_TELEOP),
                timeout_s=10.0,
                failure_message="mode manager did not enter MODE_TELEOP",
            )

            # Phase 1: approved flow → adapter gate open
            self._stream_until(
                executor=executor,
                teleop_pub=teleop_pub,
                check=lambda: (
                    self._has_open_gate(adapter_diagnostics)
                    and any(h.frame_id == robot_id for h in heartbeats)
                ),
                timeout_s=10.0,
                failure_message=(
                    "adapter gate did not open or hardware heartbeat absent"
                    f"; adapter_diagnostics={len(adapter_diagnostics)},"
                    f" heartbeats={len(heartbeats)}"
                ),
            )

            open_samples = [
                s for s in adapter_diagnostics if s.message == "approved_command"
            ]
            self.assertTrue(open_samples, "no OK adapter diagnostic with open gate")
            for status in open_samples[:5]:
                values = {kv.key: kv.value for kv in status.values}
                for key in REQUIRED_IDENTITY_KEYS:
                    self.assertEqual(
                        values.get(key),
                        IDENTITY[key],
                        f"identity key {key!r} missing or mismatched on open-gate"
                        f" status: got={values.get(key)!r}, expected={IDENTITY[key]!r}",
                    )

            # Phase 2: MRM → adapter gate closes
            adapter_diagnostics.clear()

            trigger_request = TriggerMRM.Request()
            trigger_request.requester = "adapter_template_launch_test"
            trigger_request.reason = "test_adapter_gate_closes"
            trigger_future = trigger_client.call_async(trigger_request)
            deadline = time.time() + 10.0
            while time.time() < deadline and not trigger_future.done():
                executor.spin_once(timeout_sec=0.05)
            self.assertTrue(trigger_future.done(), "trigger_mrm call did not complete")
            self.assertTrue(
                trigger_future.result().accepted,
                "trigger_mrm rejected by safety_manager",
            )

            self._stream_until(
                executor=executor,
                teleop_pub=teleop_pub,
                check=lambda: self._has_blocked_gate(adapter_diagnostics),
                timeout_s=10.0,
                failure_message=(
                    "adapter gate did not close after MRM was triggered;"
                    f" diagnostic_count={len(adapter_diagnostics)}"
                ),
            )

            blocked = [
                s
                for s in adapter_diagnostics
                if s.message in ("safety_state_blocks_output", "mrm_state_blocks_output")
            ]
            for status in blocked[:5]:
                self.assertNotEqual(
                    status.level,
                    0,  # DiagnosticStatus.OK == 0
                    f"adapter status reported OK while gate closed: msg={status.message!r}",
                )
                values = {kv.key: kv.value for kv in status.values}
                self.assertEqual(
                    values.get("output_allowed"),
                    "false",
                    "blocked status did not flag output_allowed=false",
                )
                for key in REQUIRED_IDENTITY_KEYS:
                    self.assertEqual(
                        values.get(key),
                        IDENTITY[key],
                        f"identity key {key!r} missing on blocked-gate status",
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
    def _has_open_gate(diagnostics) -> bool:
        return any(s.message == "approved_command" for s in diagnostics)

    @staticmethod
    def _has_blocked_gate(diagnostics) -> bool:
        return any(
            s.message in ("safety_state_blocks_output", "mrm_state_blocks_output")
            for s in diagnostics
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
        request.requester = "adapter_template_launch_test"
        request.reason = "adapter_template_e2e"
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
class TestHardwareAdapterTemplateShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
