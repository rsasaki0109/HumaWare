import os
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from humaware_msgs.msg import (
    Capability,
    CapabilityRegistry,
    CommandArbitrationState,
    HealthState,
    LocomotionState,
    ModeState,
    SafetyState,
    SkillExecutionState,
)
from humaware_msgs.srv import ExecuteSkill, ListCapabilities, SetMode
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


ROBOT_ID = f"mock_launch_test_{os.getpid()}"


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


class TestMockBringupLaunch(unittest.TestCase):
    def test_runtime_topics_publish(self, robot_id):
        topics = [
            (f"/{robot_id}/mode/state", ModeState),
            (f"/{robot_id}/safety/state", SafetyState),
            (f"/{robot_id}/locomotion/state", LocomotionState),
            (f"/{robot_id}/runtime/health", HealthState),
            (f"/{robot_id}/capabilities", CapabilityRegistry),
            (f"/{robot_id}/skills/state", SkillExecutionState),
            ("/diagnostics", DiagnosticArray),
        ]

        with WaitForTopics(topics, timeout=20.0, messages_received_buffer_length=10) as waiter:
            self.assertEqual(set(), waiter.topics_not_received())

            mode = self._latest_message(waiter, f"/{robot_id}/mode/state")
            safety = self._latest_message(waiter, f"/{robot_id}/safety/state")
            locomotion = self._latest_message(waiter, f"/{robot_id}/locomotion/state")
            health = self._latest_message(waiter, f"/{robot_id}/runtime/health")
            capabilities = self._latest_message(waiter, f"/{robot_id}/capabilities")
            diagnostics = waiter.received_messages("/diagnostics")

            self.assertEqual(robot_id, mode.robot_id)
            self.assertEqual(robot_id, safety.robot_id)
            self.assertEqual(robot_id, locomotion.robot_id)
            self.assertEqual(robot_id, health.robot_id)
            self.assertEqual(robot_id, capabilities.robot_id)
            self.assertIn("stop", {capability.name for capability in capabilities.capabilities})
            self.assertIn(
                "walk_velocity",
                {capability.name for capability in capabilities.capabilities},
            )
            self.assertTrue(
                any(
                    status.name == f"{robot_id}/runtime_health"
                    for message in diagnostics
                    for status in message.status
                ),
                "diagnostics did not include the runtime health status",
            )

    def test_capability_service_lists_selected_names(self, robot_id):
        self._wait_for_capabilities_topic(robot_id)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(f"capability_registry_test_{os.getpid()}", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        try:
            client = node.create_client(ListCapabilities, f"/{robot_id}/capabilities/list")
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))

            request = ListCapabilities.Request()
            request.names = ["stop", "walk_velocity", "missing_capability"]
            future = client.call_async(request)

            deadline = time.time() + 10.0
            while time.time() < deadline and not future.done():
                executor.spin_once(timeout_sec=0.1)

            self.assertTrue(future.done(), "capability registry service did not respond")
            response = future.result()
            self.assertEqual(["missing_capability"], list(response.missing_names))
            self.assertEqual(
                {"stop", "walk_velocity"},
                {capability.name for capability in response.capabilities},
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    def test_skill_server_accepts_stop_dry_run(self, robot_id):
        self._wait_for_capabilities_topic(robot_id)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(f"skill_server_test_{os.getpid()}", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        try:
            client = node.create_client(ExecuteSkill, f"/{robot_id}/skills/execute")
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))

            request = ExecuteSkill.Request()
            request.capability_name = "stop"
            request.requester = "launch_test"
            request.reason = "smoke_test"
            request.dry_run = True
            future = client.call_async(request)

            deadline = time.time() + 10.0
            while time.time() < deadline and not future.done():
                executor.spin_once(timeout_sec=0.1)

            self.assertTrue(future.done(), "skill server service did not respond")
            response = future.result()
            self.assertTrue(response.accepted)
            self.assertEqual(SkillExecutionState.STATUS_ACCEPTED, response.status)
            self.assertTrue(response.execution_id)
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    def test_skill_walk_velocity_reaches_command_arbiter(self, robot_id):
        self._wait_for_capabilities_topic(robot_id)
        with WaitForTopics([(f"/{robot_id}/safety/state", SafetyState)], timeout=10.0):
            pass

        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(f"skill_arbiter_test_{os.getpid()}", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        approved_commands = []
        arbitration_states = []
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

            mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")
            capability_client = node.create_client(
                ListCapabilities,
                f"/{robot_id}/capabilities/list",
            )
            skill_client = node.create_client(ExecuteSkill, f"/{robot_id}/skills/execute")
            self.assertTrue(mode_client.wait_for_service(timeout_sec=10.0))
            self.assertTrue(capability_client.wait_for_service(timeout_sec=10.0))
            self.assertTrue(skill_client.wait_for_service(timeout_sec=10.0))

            teleop_response = self._call_until_accepted(
                executor,
                mode_client,
                self._teleop_mode_request,
                timeout_s=10.0,
                failure_message="mode manager did not enter teleop mode",
            )
            self.assertEqual(ModeState.MODE_TELEOP, teleop_response.active_mode)

            mode_response = self._call_until_accepted(
                executor,
                mode_client,
                self._ai_policy_mode_request,
                timeout_s=10.0,
                failure_message="mode manager did not enter AI policy mode",
            )
            self.assertEqual(ModeState.MODE_AI_POLICY, mode_response.active_mode)
            self._spin_until(
                executor,
                lambda: self._arbiter_ready_for_ai_policy(arbitration_states),
                timeout_s=5.0,
                failure_message="command arbiter did not observe AI policy mode",
            )

            capability = self._wait_for_capability_state(
                executor,
                capability_client,
                "walk_velocity",
                timeout_s=10.0,
            )
            self.assertIn(
                capability.state,
                (Capability.STATE_IDLE, Capability.STATE_DEGRADED, Capability.STATE_EXECUTING),
            )

            skill_response = self._call_until_accepted(
                executor,
                skill_client,
                self._walk_velocity_skill_request,
                timeout_s=10.0,
                failure_message="skill server did not accept walk_velocity",
            )
            self.assertEqual(SkillExecutionState.STATUS_ACCEPTED, skill_response.status)

            self._spin_until(
                executor,
                lambda: self._has_ai_policy_arbitration(arbitration_states)
                and self._has_matching_approved_command(approved_commands),
                timeout_s=5.0,
                failure_message="walk_velocity command was not approved by the arbiter",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    @staticmethod
    def _latest_message(waiter, topic):
        messages = waiter.received_messages(topic)
        assert messages, f"no messages received on {topic}"
        return messages[-1]

    @staticmethod
    def _wait_for_capabilities_topic(robot_id):
        with WaitForTopics(
            [(f"/{robot_id}/capabilities", CapabilityRegistry)],
            timeout=10.0,
            messages_received_buffer_length=1,
        ):
            pass

    @staticmethod
    def _teleop_mode_request():
        request = SetMode.Request()
        request.requested_mode = ModeState.MODE_TELEOP
        request.requester = "launch_test"
        request.reason = "skill_arbiter_e2e"
        return request

    @staticmethod
    def _ai_policy_mode_request():
        request = SetMode.Request()
        request.requested_mode = ModeState.MODE_AI_POLICY
        request.requester = "launch_test"
        request.reason = "skill_arbiter_e2e"
        return request

    @staticmethod
    def _walk_velocity_skill_request():
        request = ExecuteSkill.Request()
        request.capability_name = "walk_velocity"
        request.requester = "launch_test"
        request.reason = "skill_arbiter_e2e"
        request.dry_run = False
        request.velocity_command.header.frame_id = ROBOT_ID
        request.velocity_command.twist.linear.x = 0.2
        request.velocity_command.twist.angular.z = 0.1
        return request

    def _call_service(self, executor, client, request, timeout_s, failure_message):
        future = client.call_async(request)
        self._spin_until(executor, future.done, timeout_s, failure_message)
        return future.result()

    def _call_until_accepted(
        self,
        executor,
        client,
        request_factory,
        timeout_s,
        failure_message,
    ):
        deadline = time.time() + timeout_s
        last_message = ""
        while time.time() < deadline:
            response = self._call_service(
                executor,
                client,
                request_factory(),
                max(0.1, deadline - time.time()),
                failure_message,
            )
            if response.accepted:
                return response
            last_message = getattr(response, "message", "")
            executor.spin_once(timeout_sec=0.1)

        self.fail(f"{failure_message}; last response: {last_message}")

    def _wait_for_capability_state(self, executor, client, capability_name, timeout_s):
        deadline = time.time() + timeout_s
        last_state = Capability.STATE_UNKNOWN
        while time.time() < deadline:
            request = ListCapabilities.Request()
            request.names = [capability_name]
            response = self._call_service(
                executor,
                client,
                request,
                max(0.1, deadline - time.time()),
                f"capability registry did not respond for {capability_name}",
            )
            if response.capabilities:
                capability = response.capabilities[0]
                last_state = capability.state
                if capability.state not in (
                    Capability.STATE_UNKNOWN,
                    Capability.STATE_UNAVAILABLE,
                    Capability.STATE_FAULT,
                ):
                    return capability
            executor.spin_once(timeout_sec=0.1)

        self.fail(f"{capability_name} did not become available; last state: {last_state}")

    def _spin_until(self, executor, predicate, timeout_s, failure_message):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if predicate():
                return
            executor.spin_once(timeout_sec=0.05)
        if predicate():
            return
        self.fail(failure_message)

    @staticmethod
    def _arbiter_ready_for_ai_policy(states):
        return any(
            state.active_mode == ModeState.MODE_AI_POLICY
            and state.safety_state in (SafetyState.STATE_OK, SafetyState.STATE_WARN)
            for state in states
        )

    @staticmethod
    def _has_ai_policy_arbitration(states):
        return any(
            state.output_enabled
            and state.active_source == CommandArbitrationState.SOURCE_AI_POLICY
            and state.reason == "approved"
            for state in states
        )

    @staticmethod
    def _has_matching_approved_command(commands):
        return any(
            abs(command.twist.linear.x - 0.2) < 1e-6
            and abs(command.twist.angular.z - 0.1) < 1e-6
            for command in commands
        )


@launch_testing.post_shutdown_test()
class TestMockBringupShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
