"""Launch tests verifying teleop and autonomy candidate command approval."""

import os
import time
import unittest

from geometry_msgs.msg import TwistStamped
from humaware_msgs.msg import (
    CommandArbitrationState,
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


ROBOT_ID = f"mock_candidate_paths_{os.getpid()}"


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


class TestCandidateCommandPaths(unittest.TestCase):
    def test_teleop_candidate_command_is_approved(self, robot_id):
        self._run_candidate_path_test(
            robot_id=robot_id,
            target_mode=ModeState.MODE_TELEOP,
            candidate_topic=f"/{robot_id}/teleop/cmd_vel",
            expected_source=CommandArbitrationState.SOURCE_TELEOP,
            linear_x=0.15,
            angular_z=0.0,
        )

    def test_autonomy_candidate_command_is_approved(self, robot_id):
        self._run_candidate_path_test(
            robot_id=robot_id,
            target_mode=ModeState.MODE_AUTONOMY,
            candidate_topic=f"/{robot_id}/autonomy/cmd_vel",
            expected_source=CommandArbitrationState.SOURCE_AUTONOMY,
            linear_x=0.1,
            angular_z=0.2,
        )

    def _run_candidate_path_test(
        self,
        robot_id: str,
        target_mode: int,
        candidate_topic: str,
        expected_source: int,
        linear_x: float,
        angular_z: float,
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
            f"candidate_path_test_{expected_source}_{os.getpid()}",
            context=context,
        )
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
            candidate_pub = node.create_publisher(TwistStamped, candidate_topic, 10)

            mode_client = node.create_client(SetMode, f"/{robot_id}/mode/set")
            self.assertTrue(mode_client.wait_for_service(timeout_sec=10.0))

            self._call_until_accepted(
                executor=executor,
                client=mode_client,
                request_factory=lambda: self._mode_request(target_mode),
                timeout_s=10.0,
                failure_message=f"mode manager did not enter mode {target_mode}",
            )

            def candidate():
                msg = TwistStamped()
                msg.header.frame_id = robot_id
                msg.twist.linear.x = linear_x
                msg.twist.angular.z = angular_z
                return msg

            deadline = time.time() + 8.0
            while time.time() < deadline and not self._approved_matches(
                approved_commands, arbitration_states, expected_source, linear_x, angular_z
            ):
                candidate_pub.publish(candidate())
                executor.spin_once(timeout_sec=0.05)

            self.assertTrue(
                self._approved_matches(
                    approved_commands,
                    arbitration_states,
                    expected_source,
                    linear_x,
                    angular_z,
                ),
                "candidate command was not approved by the arbiter",
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
        request.requester = "candidate_path_launch_test"
        request.reason = "candidate_path_e2e"
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

    @staticmethod
    def _approved_matches(
        approved_commands,
        arbitration_states,
        expected_source: int,
        linear_x: float,
        angular_z: float,
    ) -> bool:
        has_approval_state = any(
            state.output_enabled
            and state.active_source == expected_source
            and state.reason == "approved"
            for state in arbitration_states
        )
        has_command = any(
            abs(command.twist.linear.x - linear_x) < 1e-6
            and abs(command.twist.angular.z - angular_z) < 1e-6
            for command in approved_commands
        )
        return has_approval_state and has_command


@launch_testing.post_shutdown_test()
class TestCandidatePathsShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
