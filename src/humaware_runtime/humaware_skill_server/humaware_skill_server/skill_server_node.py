"""Capability-gated skill server for HumaWare."""

from dataclasses import dataclass
from uuid import uuid4

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from humaware_msgs.msg import (
    Capability,
    CapabilityRegistry,
    ModeState,
    SafetyState,
    SkillExecutionState,
)
from humaware_msgs.srv import ExecuteSkill


EXECUTABLE_SKILLS = {"stop", "walk_velocity", "turn_in_place"}


@dataclass(frozen=True)
class SkillDecision:
    """Decision returned before a skill request is executed."""

    accepted: bool
    message: str
    status: int


class SkillServerNode(Node):
    """Validate skill requests against capabilities before publishing candidates."""

    def __init__(self) -> None:
        super().__init__("skill_server")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("publish_rate_hz", 1.0)

        self._capabilities: dict[str, Capability] = {}
        self._mode = ModeState.MODE_UNKNOWN
        self._safety = SafetyState.STATE_UNKNOWN
        self._last_state: SkillExecutionState | None = None

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._state_pub = self.create_publisher(SkillExecutionState, "skills/state", qos)
        self._policy_cmd_pub = self.create_publisher(TwistStamped, "policy/cmd_vel", 10)

        self.create_subscription(CapabilityRegistry, "capabilities", self._on_capabilities, qos)
        self.create_subscription(ModeState, "mode/state", self._on_mode_state, 10)
        self.create_subscription(SafetyState, "safety/state", self._on_safety_state, 10)

        self.create_service(ExecuteSkill, "skills/execute", self._on_execute_skill)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(publish_rate_hz, 0.1), self._republish_state)
        self._publish_state(
            execution_id="",
            capability=None,
            capability_name="",
            requester="",
            status=SkillExecutionState.STATUS_UNKNOWN,
            message="waiting_for_skill_request",
            command_published=False,
        )

    @property
    def robot_id(self) -> str:
        return str(self.get_parameter("robot_id").value)

    def _on_capabilities(self, msg: CapabilityRegistry) -> None:
        self._capabilities = {capability.name: capability for capability in msg.capabilities}

    def _on_mode_state(self, msg: ModeState) -> None:
        self._mode = msg.active_mode

    def _on_safety_state(self, msg: SafetyState) -> None:
        self._safety = msg.state

    def _on_execute_skill(self, request, response):
        execution_id = str(uuid4())
        capability = self._capabilities.get(request.capability_name)
        decision = evaluate_request(
            request.capability_name,
            capability,
            self._mode,
            self._safety,
        )

        command_published = False
        if decision.accepted and not request.dry_run:
            self._execute_supported_skill(request)
            command_published = request.capability_name in EXECUTABLE_SKILLS

        self._publish_state(
            execution_id=execution_id,
            capability=capability,
            capability_name=request.capability_name,
            requester=request.requester,
            status=decision.status,
            message=decision.message,
            command_published=command_published,
        )

        response.accepted = decision.accepted
        response.message = decision.message
        response.execution_id = execution_id
        response.status = decision.status
        return response

    def _execute_supported_skill(self, request) -> None:
        if request.capability_name == "stop":
            self._policy_cmd_pub.publish(make_stop_command(self.robot_id, self.get_clock().now()))
            return

        if request.capability_name == "walk_velocity":
            command = sanitize_velocity_command(
                request.velocity_command,
                self.robot_id,
                self.get_clock().now(),
                turn_only=False,
            )
            self._policy_cmd_pub.publish(command)
            return

        if request.capability_name == "turn_in_place":
            command = sanitize_velocity_command(
                request.velocity_command,
                self.robot_id,
                self.get_clock().now(),
                turn_only=True,
            )
            self._policy_cmd_pub.publish(command)

    def _publish_state(
        self,
        execution_id: str,
        capability: Capability | None,
        capability_name: str,
        requester: str,
        status: int,
        message: str,
        command_published: bool,
    ) -> None:
        state = SkillExecutionState()
        now = self.get_clock().now()
        state.header.stamp = now.to_msg()
        state.header.frame_id = self.robot_id
        state.robot_id = self.robot_id
        state.execution_id = execution_id
        state.capability_name = capability_name
        state.requester = requester
        state.status = status
        state.message = message
        state.accepted_at = now.to_msg()
        if capability is not None:
            state.timeout = capability.timeout
        state.command_published = command_published
        self._last_state = state
        self._state_pub.publish(state)

    def _republish_state(self) -> None:
        if self._last_state is not None:
            self._last_state.header.stamp = self.get_clock().now().to_msg()
            self._state_pub.publish(self._last_state)


def evaluate_request(
    capability_name: str,
    capability: Capability | None,
    mode: int,
    safety: int,
) -> SkillDecision:
    if capability is None:
        return SkillDecision(False, "capability_not_found", SkillExecutionState.STATUS_REJECTED)

    if capability_name not in EXECUTABLE_SKILLS:
        return SkillDecision(False, "no_skill_handler", SkillExecutionState.STATUS_REJECTED)

    if capability_name != "stop" and safety in unsafe_states():
        return SkillDecision(False, "safety_state_blocks_skill", SkillExecutionState.STATUS_REJECTED)

    if capability_name in {"walk_velocity", "turn_in_place"} and mode != ModeState.MODE_AI_POLICY:
        return SkillDecision(False, "skill_requires_ai_policy_mode", SkillExecutionState.STATUS_REJECTED)

    if capability.state in (Capability.STATE_UNKNOWN, Capability.STATE_UNAVAILABLE, Capability.STATE_FAULT):
        return SkillDecision(False, "capability_not_available", SkillExecutionState.STATUS_REJECTED)

    return SkillDecision(True, "accepted", SkillExecutionState.STATUS_ACCEPTED)


def unsafe_states() -> set[int]:
    return {SafetyState.STATE_FAULT, SafetyState.STATE_ESTOP, SafetyState.STATE_MRM}


def make_stop_command(robot_id: str, now) -> TwistStamped:
    msg = TwistStamped()
    msg.header.stamp = now.to_msg()
    msg.header.frame_id = robot_id
    return msg


def sanitize_velocity_command(
    request: TwistStamped,
    robot_id: str,
    now,
    turn_only: bool,
) -> TwistStamped:
    msg = TwistStamped()
    msg.header.stamp = now.to_msg()
    msg.header.frame_id = request.header.frame_id or robot_id
    if not turn_only:
        msg.twist.linear.x = request.twist.linear.x
        msg.twist.linear.y = request.twist.linear.y
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = request.twist.angular.z
    return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SkillServerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
