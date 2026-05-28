"""Mock locomotion adapter for HumaWare."""

from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from humaware_msgs.msg import LocomotionState, ModeState, SafetyState


class MockLocomotionAdapterNode(Node):
    """Translate approved velocity commands into mock locomotion state."""

    def __init__(self) -> None:
        super().__init__("mock_locomotion_adapter")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.75)
        self.declare_parameter("stop_settle_time_s", 0.4)
        self.declare_parameter("walk_epsilon", 0.01)
        self.declare_parameter("max_safe_linear_velocity_mps", 0.5)
        self.declare_parameter("max_safe_angular_velocity_radps", 0.5)

        self._active_mode = ModeState.MODE_INACTIVE
        self._safety_state = SafetyState.STATE_UNKNOWN
        self._last_command_time: Optional[Time] = None
        self._stop_started_at: Optional[Time] = None
        self._state = LocomotionState.STATE_INACTIVE
        self._linear_velocity_mps = 0.0
        self._angular_velocity_radps = 0.0
        self._constraints = ["waiting_for_mode", "waiting_for_safety_state"]

        self._state_pub = self.create_publisher(LocomotionState, "locomotion/state", 10)
        self.create_subscription(TwistStamped, "cmd_vel/approved", self._on_approved_cmd, 10)
        self.create_subscription(ModeState, "mode/state", self._on_mode_state, 10)
        self.create_subscription(SafetyState, "safety/state", self._on_safety_state, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._tick)

    def _on_mode_state(self, msg: ModeState) -> None:
        self._active_mode = msg.active_mode

    def _on_safety_state(self, msg: SafetyState) -> None:
        self._safety_state = msg.state

    def _on_approved_cmd(self, msg: TwistStamped) -> None:
        now = self.get_clock().now()
        self._last_command_time = now

        linear = float(msg.twist.linear.x)
        angular = float(msg.twist.angular.z)
        epsilon = float(self.get_parameter("walk_epsilon").value)

        if abs(linear) <= epsilon and abs(angular) <= epsilon:
            self._linear_velocity_mps = 0.0
            self._angular_velocity_radps = 0.0
            self._begin_stop(now, "approved_stop_command")
            return

        if self._is_output_blocked():
            self._linear_velocity_mps = 0.0
            self._angular_velocity_radps = 0.0
            self._begin_stop(now, "runtime_blocks_output")
            return

        self._stop_started_at = None
        self._linear_velocity_mps = linear
        self._angular_velocity_radps = angular
        if abs(linear) > epsilon:
            self._state = LocomotionState.STATE_WALKING
        else:
            self._state = LocomotionState.STATE_TURNING
        self._constraints = []

    def _tick(self) -> None:
        now = self.get_clock().now()
        self._update_state_for_runtime(now)
        self._publish_state(now)

    def _update_state_for_runtime(self, now: Time) -> None:
        if self._active_mode in (
            ModeState.MODE_INACTIVE,
            ModeState.MODE_MAINTENANCE,
        ):
            self._linear_velocity_mps = 0.0
            self._angular_velocity_radps = 0.0
            self._state = LocomotionState.STATE_INACTIVE
            self._constraints = ["mode_inactive"]
            self._stop_started_at = None
            return

        if self._active_mode in (
            ModeState.MODE_FAULT,
            ModeState.MODE_SHUTDOWN,
        ):
            self._linear_velocity_mps = 0.0
            self._angular_velocity_radps = 0.0
            self._begin_stop(now, "mode_blocks_locomotion")
            return

        if self._safety_state in (
            SafetyState.STATE_UNKNOWN,
            SafetyState.STATE_FAULT,
            SafetyState.STATE_ESTOP,
            SafetyState.STATE_MRM,
        ):
            self._linear_velocity_mps = 0.0
            self._angular_velocity_radps = 0.0
            self._begin_stop(now, "safety_blocks_locomotion")
            return

        if self._last_command_time is None:
            self._state = LocomotionState.STATE_STANDING
            self._constraints = ["waiting_for_command"]
            return

        timeout_s = float(self.get_parameter("command_timeout_s").value)
        if now - self._last_command_time > Duration(seconds=timeout_s):
            self._linear_velocity_mps = 0.0
            self._angular_velocity_radps = 0.0
            self._begin_stop(now, "command_timeout")
            return

        self._settle_stop_if_ready(now)

    def _begin_stop(self, now: Time, reason: str) -> None:
        if self._active_mode in (
            ModeState.MODE_INACTIVE,
            ModeState.MODE_MAINTENANCE,
        ):
            self._state = LocomotionState.STATE_INACTIVE
            self._constraints = ["mode_inactive"]
            self._stop_started_at = None
            return

        if self._state not in (
            LocomotionState.STATE_STOPPING,
            LocomotionState.STATE_STANDING,
        ):
            self._stop_started_at = now
        elif self._stop_started_at is None:
            self._stop_started_at = now

        self._state = LocomotionState.STATE_STOPPING
        self._constraints = [reason]
        self._settle_stop_if_ready(now)

    def _settle_stop_if_ready(self, now: Time) -> None:
        if self._state != LocomotionState.STATE_STOPPING:
            return
        if self._stop_started_at is None:
            return
        settle_s = float(self.get_parameter("stop_settle_time_s").value)
        if now - self._stop_started_at >= Duration(seconds=settle_s):
            self._state = LocomotionState.STATE_STANDING
            # Clear the stop reason only when the robot is free to move again.
            # When locomotion is still inhibited (safety not OK/WARN, or a
            # non-motion mode), the robot is being *held* standing -- keep the
            # holding reason that _begin_stop set so the state keeps explaining
            # why it will not move, instead of reporting an empty constraint
            # list that looks indistinguishable from a healthy idle robot.
            if not self._is_output_blocked():
                self._constraints = []

    def _is_output_blocked(self) -> bool:
        return self._active_mode not in (
            ModeState.MODE_TELEOP,
            ModeState.MODE_AUTONOMY,
            ModeState.MODE_AI_POLICY,
        ) or self._safety_state not in (
            SafetyState.STATE_OK,
            SafetyState.STATE_WARN,
        )

    def _publish_state(self, now: Time) -> None:
        robot_id = str(self.get_parameter("robot_id").value)
        state = LocomotionState()
        state.header.stamp = now.to_msg()
        state.header.frame_id = robot_id
        state.robot_id = robot_id
        state.state = self._state
        state.active_adapter = "mock"
        state.commanded_linear_velocity_mps = self._linear_velocity_mps
        state.commanded_angular_velocity_radps = self._angular_velocity_radps
        state.max_safe_linear_velocity_mps = float(
            self.get_parameter("max_safe_linear_velocity_mps").value
        )
        state.max_safe_angular_velocity_radps = float(
            self.get_parameter("max_safe_angular_velocity_radps").value
        )
        state.balance_required = True
        state.navigation_control_active = self._active_mode == ModeState.MODE_AUTONOMY
        state.active_constraints = self._constraints
        self._state_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockLocomotionAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
