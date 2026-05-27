"""Minimal HumaWare policy provider stub.

The stub publishes scripted candidate velocity commands onto
``policy/cmd_vel`` so the rest of the runtime (mode manager, safety
manager, command arbiter) can be exercised end-to-end without a real
policy. It only emits while AI policy mode is active and safety state
allows it. The stub never publishes to approved or actuator topics and
never bypasses ``humaware_safety_manager`` or
``humaware_command_arbiter``.
"""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from humaware_msgs.msg import ModeState, SafetyState


SAFE_FOR_POLICY = (SafetyState.STATE_OK, SafetyState.STATE_WARN)


def should_emit_candidate(
    enabled: bool,
    active_mode: int,
    safety_state: int,
    safety_seen: bool,
) -> tuple[bool, str]:
    """Return (allow, reason) for the policy candidate-action gate.

    The stub publishes a candidate velocity command only when every
    condition holds. The function is pure to keep the policy gate
    testable; the node simply consults it on every tick.
    """
    if not enabled:
        return False, "policy_disabled_by_parameter"
    if not safety_seen:
        return False, "waiting_for_safety_state"
    if safety_state not in SAFE_FOR_POLICY:
        return False, "safety_state_blocks_policy"
    if active_mode != ModeState.MODE_AI_POLICY:
        return False, "active_mode_not_ai_policy"
    return True, "policy_candidate_emitted"


class PolicyProviderStubNode(Node):
    """Publish scripted candidate commands onto policy/cmd_vel."""

    def __init__(self) -> None:
        super().__init__("policy_provider_stub")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("enabled", False)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("linear_x_mps", 0.05)
        self.declare_parameter("angular_z_radps", 0.0)
        self.declare_parameter("provider_source", "stub")
        self.declare_parameter("confidence", 0.5)

        self._active_mode = ModeState.MODE_INACTIVE
        self._safety_state = SafetyState.STATE_UNKNOWN
        self._safety_seen = False

        self._candidate_pub = self.create_publisher(TwistStamped, "policy/cmd_vel", 10)
        self.create_subscription(ModeState, "mode/state", self._on_mode, 10)
        self.create_subscription(SafetyState, "safety/state", self._on_safety, 10)

        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(period, self._tick)

    def _on_mode(self, msg: ModeState) -> None:
        self._active_mode = msg.active_mode

    def _on_safety(self, msg: SafetyState) -> None:
        self._safety_state = msg.state
        self._safety_seen = True

    def _tick(self) -> None:
        allow, reason = should_emit_candidate(
            enabled=bool(self.get_parameter("enabled").value),
            active_mode=self._active_mode,
            safety_state=self._safety_state,
            safety_seen=self._safety_seen,
        )
        if not allow:
            self.get_logger().debug(
                f"policy stub idle: {reason}"
                f" provider={self.get_parameter('provider_source').value}"
            )
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("robot_id").value)
        msg.twist.linear.x = float(self.get_parameter("linear_x_mps").value)
        msg.twist.angular.z = float(self.get_parameter("angular_z_radps").value)
        self._candidate_pub.publish(msg)
        self.get_logger().debug(
            f"policy stub emitted candidate: linear.x={msg.twist.linear.x:.3f}"
            f" angular.z={msg.twist.angular.z:.3f}"
            f" provider={self.get_parameter('provider_source').value}"
            f" confidence={self.get_parameter('confidence').value}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyProviderStubNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
