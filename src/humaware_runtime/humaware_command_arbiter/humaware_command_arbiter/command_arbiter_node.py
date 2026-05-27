"""Candidate velocity command arbiter for HumaWare."""

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from humaware_msgs.msg import CommandArbitrationState, ModeState, SafetyState


@dataclass
class CandidateCommand:
    """Last command received from a source."""

    source: int
    name: str
    required_mode: int
    msg: TwistStamped
    received_at: Time


class CommandArbiterNode(Node):
    """Gate candidate velocity commands by priority, mode, timeout, and safety."""

    def __init__(self) -> None:
        super().__init__("command_arbiter")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("max_linear_velocity_mps", 0.5)
        self.declare_parameter("max_angular_velocity_radps", 0.5)
        self.declare_parameter("publish_stop_on_block", True)

        self._active_mode = ModeState.MODE_INACTIVE
        self._safety_state = SafetyState.STATE_UNKNOWN
        self._last_safety: Optional[SafetyState] = None
        self._candidates: Dict[int, CandidateCommand] = {}

        self._approved_pub = self.create_publisher(TwistStamped, "cmd_vel/approved", 10)
        self._state_pub = self.create_publisher(
            CommandArbitrationState,
            "runtime/command_arbitration_state",
            10,
        )

        self.create_subscription(ModeState, "mode/state", self._on_mode_state, 10)
        self.create_subscription(SafetyState, "safety/state", self._on_safety_state, 10)
        self.create_subscription(
            TwistStamped,
            "teleop/cmd_vel",
            lambda msg: self._on_candidate(
                CommandArbitrationState.SOURCE_TELEOP,
                "teleop",
                ModeState.MODE_TELEOP,
                msg,
            ),
            10,
        )
        self.create_subscription(
            TwistStamped,
            "autonomy/cmd_vel",
            lambda msg: self._on_candidate(
                CommandArbitrationState.SOURCE_AUTONOMY,
                "autonomy",
                ModeState.MODE_AUTONOMY,
                msg,
            ),
            10,
        )
        self.create_subscription(
            TwistStamped,
            "policy/cmd_vel",
            lambda msg: self._on_candidate(
                CommandArbitrationState.SOURCE_AI_POLICY,
                "ai_policy",
                ModeState.MODE_AI_POLICY,
                msg,
            ),
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._tick)

    def _on_mode_state(self, msg: ModeState) -> None:
        self._active_mode = msg.active_mode

    def _on_safety_state(self, msg: SafetyState) -> None:
        self._last_safety = msg
        self._safety_state = msg.state

    def _on_candidate(self, source: int, name: str, required_mode: int, msg: TwistStamped) -> None:
        self._candidates[source] = CandidateCommand(
            source=source,
            name=name,
            required_mode=required_mode,
            msg=msg,
            received_at=self.get_clock().now(),
        )

    def _tick(self) -> None:
        now = self.get_clock().now()
        candidate, reason = self._select_candidate(now)

        if candidate is None:
            if self._should_publish_stop(reason):
                self._approved_pub.publish(self._make_stop(now))
                self._publish_state(
                    now,
                    active_source=CommandArbitrationState.SOURCE_SAFETY,
                    active_source_name="safety",
                    output_enabled=False,
                    stop_command_published=True,
                    reason=reason,
                )
            else:
                self._publish_state(
                    now,
                    active_source=CommandArbitrationState.SOURCE_NONE,
                    active_source_name="none",
                    output_enabled=False,
                    stop_command_published=False,
                    reason=reason,
                )
            return

        approved = self._limit_command(candidate.msg, now)
        self._approved_pub.publish(approved)
        self._publish_state(
            now,
            active_source=candidate.source,
            active_source_name=candidate.name,
            output_enabled=True,
            stop_command_published=False,
            reason="approved",
            last_command_time=candidate.received_at,
        )

    def _select_candidate(self, now: Time):
        if self._last_safety is None:
            return None, "waiting_for_safety_state"
        if self._safety_state in (
            SafetyState.STATE_FAULT,
            SafetyState.STATE_ESTOP,
            SafetyState.STATE_MRM,
        ):
            return None, "safety_state_blocks_output"
        if self._safety_state not in (SafetyState.STATE_OK, SafetyState.STATE_WARN):
            return None, "safety_state_not_ready"
        if self._active_mode in (
            ModeState.MODE_INACTIVE,
            ModeState.MODE_MAINTENANCE,
            ModeState.MODE_FAULT,
            ModeState.MODE_SHUTDOWN,
        ):
            return None, "active_mode_blocks_output"

        for source in (
            CommandArbitrationState.SOURCE_TELEOP,
            CommandArbitrationState.SOURCE_AUTONOMY,
            CommandArbitrationState.SOURCE_AI_POLICY,
        ):
            candidate = self._candidates.get(source)
            if candidate is None:
                continue
            if candidate.required_mode != self._active_mode:
                continue
            if self._is_fresh(candidate, now):
                return candidate, "approved"

        return None, "no_fresh_command_for_active_mode"

    def _is_fresh(self, candidate: CandidateCommand, now: Time) -> bool:
        timeout_s = float(self.get_parameter("command_timeout_s").value)
        return now - candidate.received_at <= Duration(seconds=timeout_s)

    def _should_publish_stop(self, reason: str) -> bool:
        if not bool(self.get_parameter("publish_stop_on_block").value):
            return False
        return reason in (
            "safety_state_blocks_output",
            "active_mode_blocks_output",
            "no_fresh_command_for_active_mode",
        )

    def _limit_command(self, msg: TwistStamped, now: Time) -> TwistStamped:
        max_linear = float(self.get_parameter("max_linear_velocity_mps").value)
        max_angular = float(self.get_parameter("max_angular_velocity_radps").value)

        approved = TwistStamped()
        approved.header.stamp = now.to_msg()
        approved.header.frame_id = msg.header.frame_id
        approved.twist.linear.x = self._clamp(msg.twist.linear.x, -max_linear, max_linear)
        approved.twist.linear.y = self._clamp(msg.twist.linear.y, -max_linear, max_linear)
        approved.twist.linear.z = 0.0
        approved.twist.angular.x = 0.0
        approved.twist.angular.y = 0.0
        approved.twist.angular.z = self._clamp(msg.twist.angular.z, -max_angular, max_angular)
        return approved

    def _make_stop(self, now: Time) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = str(self.get_parameter("robot_id").value)
        return msg

    def _publish_state(
        self,
        now: Time,
        active_source: int,
        active_source_name: str,
        output_enabled: bool,
        stop_command_published: bool,
        reason: str,
        last_command_time: Optional[Time] = None,
    ) -> None:
        state = CommandArbitrationState()
        state.header.stamp = now.to_msg()
        state.header.frame_id = str(self.get_parameter("robot_id").value)
        state.robot_id = str(self.get_parameter("robot_id").value)
        state.active_source = active_source
        state.active_source_name = active_source_name
        state.active_mode = self._active_mode
        state.safety_state = self._safety_state
        state.output_enabled = output_enabled
        state.stop_command_published = stop_command_published
        if last_command_time is not None:
            state.last_command_time = last_command_time.to_msg()
            state.command_age = self._duration_msg(now - last_command_time)
        state.reason = reason
        self._state_pub.publish(state)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _duration_msg(duration: Duration) -> DurationMsg:
        msg = DurationMsg()
        total_nanoseconds = duration.nanoseconds
        msg.sec = int(total_nanoseconds // 1_000_000_000)
        msg.nanosec = int(total_nanoseconds % 1_000_000_000)
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandArbiterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
