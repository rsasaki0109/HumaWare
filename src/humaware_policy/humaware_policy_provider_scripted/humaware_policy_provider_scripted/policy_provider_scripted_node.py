"""HumaWare scripted policy provider node.

The node loads a YAML waypoint plan and emits candidate velocity
commands onto ``policy/cmd_vel`` while AI policy mode is active and the
safety state allows it. The gate check is intentionally identical in
spirit to :mod:`humaware_policy_provider_stub` — every provider must
satisfy the same candidate-action contract before publishing.
"""

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from humaware_msgs.msg import ModeState, SafetyState

from humaware_policy_provider_scripted.plan import (
    PlanProgress,
    WaypointPlan,
    advance_progress,
    current_command,
    initial_progress,
    parse_plan,
)


SAFE_FOR_POLICY = (SafetyState.STATE_OK, SafetyState.STATE_WARN)


def should_emit_candidate(
    enabled: bool,
    active_mode: int,
    safety_state: int,
    safety_seen: bool,
) -> tuple[bool, str]:
    """Return ``(allow, reason)`` for the candidate-action gate.

    Mirrors :func:`humaware_policy_provider_stub.policy_provider_stub_node.
    should_emit_candidate`. Duplicated here to avoid inter-provider
    package coupling — the contract is canonical, not the implementation.
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


class PolicyProviderScriptedNode(Node):
    """Walk a scripted waypoint plan, gated by mode and safety state."""

    def __init__(self) -> None:
        super().__init__("policy_provider_scripted")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("enabled", False)
        self.declare_parameter("plan_yaml_path", "")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("provider_source", "scripted")
        self.declare_parameter("confidence", 0.5)

        self._active_mode = ModeState.MODE_INACTIVE
        self._safety_state = SafetyState.STATE_UNKNOWN
        self._safety_seen = False

        self._plan = self._load_plan()
        self._progress: PlanProgress = initial_progress()

        self._candidate_pub = self.create_publisher(TwistStamped, "policy/cmd_vel", 10)
        self.create_subscription(ModeState, "mode/state", self._on_mode, 10)
        self.create_subscription(SafetyState, "safety/state", self._on_safety, 10)

        self._publish_rate = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(1.0 / self._publish_rate, self._tick)

    def _load_plan(self) -> WaypointPlan:
        path = str(self.get_parameter("plan_yaml_path").value)
        if not path:
            self.get_logger().warn(
                "plan_yaml_path is empty; provider will stay idle even when gated open"
            )
            return WaypointPlan(waypoints=(), loop=False)
        try:
            with open(path) as handle:
                raw = yaml.safe_load(handle) or {}
            plan = parse_plan(raw)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(
                f"failed to load scripted plan from {path}: {exc}; staying idle"
            )
            return WaypointPlan(waypoints=(), loop=False)
        self.get_logger().info(
            f"loaded {len(plan.waypoints)} waypoints from {path}, loop={plan.loop}"
        )
        return plan

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
        provider = str(self.get_parameter("provider_source").value)
        if not allow:
            self.get_logger().debug(
                f"scripted policy idle: {reason} provider={provider}"
            )
            return

        dt = 1.0 / self._publish_rate
        self._progress = advance_progress(self._progress, self._plan, dt)
        waypoint = current_command(self._progress, self._plan)
        if waypoint is None:
            self.get_logger().debug(
                f"scripted policy idle: plan_completed provider={provider}"
            )
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("robot_id").value)
        msg.twist.linear.x = waypoint.linear_x_mps
        msg.twist.angular.z = waypoint.angular_z_radps
        self._candidate_pub.publish(msg)
        self.get_logger().debug(
            f"scripted policy emitted candidate: linear.x={waypoint.linear_x_mps:.3f}"
            f" angular.z={waypoint.angular_z_radps:.3f}"
            f" index={self._progress.waypoint_index}"
            f" provider={provider}"
            f" confidence={self.get_parameter('confidence').value}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyProviderScriptedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
