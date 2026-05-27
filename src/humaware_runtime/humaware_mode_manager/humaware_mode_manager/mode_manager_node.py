"""Runtime mode manager for HumaWare."""

from typing import Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from humaware_msgs.msg import ModeState, ModeTransitionState, SafetyState
from humaware_msgs.srv import SetMode, Takeover


MODE_NAMES: Dict[str, int] = {
    "maintenance": ModeState.MODE_MAINTENANCE,
    "inactive": ModeState.MODE_INACTIVE,
    "teleop": ModeState.MODE_TELEOP,
    "autonomy": ModeState.MODE_AUTONOMY,
    "ai_policy": ModeState.MODE_AI_POLICY,
    "fault": ModeState.MODE_FAULT,
    "shutdown": ModeState.MODE_SHUTDOWN,
}

MODE_VALUES = set(MODE_NAMES.values())
ACTIVE_MODES = (
    ModeState.MODE_TELEOP,
    ModeState.MODE_AUTONOMY,
    ModeState.MODE_AI_POLICY,
)
AUTONOMY_MODES = (
    ModeState.MODE_AUTONOMY,
    ModeState.MODE_AI_POLICY,
)
AI_POLICY_ENTRY_MODES = (
    ModeState.MODE_TELEOP,
    ModeState.MODE_AUTONOMY,
)
AUTONOMY_BLOCKING_SAFETY_STATES = (
    SafetyState.STATE_UNKNOWN,
    SafetyState.STATE_FAULT,
    SafetyState.STATE_ESTOP,
    SafetyState.STATE_MRM,
)


class ModeManagerNode(Node):
    """Publish mode state and handle validated mode transition requests."""

    def __init__(self) -> None:
        super().__init__("mode_manager")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("initial_mode", "inactive")
        self.declare_parameter("publish_rate_hz", 10.0)

        initial_mode = self._parse_initial_mode()
        self._requested_mode = initial_mode
        self._active_mode = initial_mode
        self._owner = "mode_manager"
        self._transition_reason = "initial_mode"
        self._active_since = self.get_clock().now().to_msg()
        self._safety_state = SafetyState.STATE_UNKNOWN
        self._safety_seen = False

        self._mode_pub = self.create_publisher(ModeState, "mode/state", 10)
        self._transition_pub = self.create_publisher(
            ModeTransitionState,
            "mode/transition_state",
            10,
        )
        self._set_mode_srv = self.create_service(SetMode, "mode/set", self._handle_set_mode)
        self._takeover_srv = self.create_service(Takeover, "mode/takeover", self._handle_takeover)
        self.create_subscription(SafetyState, "safety/state", self._on_safety_state, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._publish_state)

    def _parse_initial_mode(self) -> int:
        initial_mode = self.get_parameter("initial_mode").value
        if isinstance(initial_mode, str):
            return MODE_NAMES.get(initial_mode.lower(), ModeState.MODE_INACTIVE)
        if int(initial_mode) in MODE_VALUES:
            return int(initial_mode)
        return ModeState.MODE_INACTIVE

    def _handle_set_mode(self, request: SetMode.Request, response: SetMode.Response):
        if request.requested_mode not in MODE_VALUES:
            response.accepted = False
            response.active_mode = self._active_mode
            response.message = f"unsupported mode: {request.requested_mode}"
            self._publish_transition(
                previous_mode=self._active_mode,
                requested_mode=request.requested_mode,
                requester=request.requester or "anonymous",
                reason=request.reason or "mode_set",
                outcome=ModeTransitionState.OUTCOME_REJECTED,
                message=response.message,
                takeover=False,
            )
            return response

        requester = request.requester or "anonymous"
        reason = request.reason or "mode_set"
        accepted, message = self._evaluate_transition(request.requested_mode, takeover=False)
        previous_mode = self._active_mode

        if not accepted:
            self._requested_mode = request.requested_mode
            response.accepted = False
            response.active_mode = self._active_mode
            response.message = message
            self._publish_transition(
                previous_mode=previous_mode,
                requested_mode=request.requested_mode,
                requester=requester,
                reason=reason,
                outcome=ModeTransitionState.OUTCOME_REJECTED,
                message=message,
                takeover=False,
            )
            return response

        self._apply_mode(request.requested_mode, requester, reason)

        response.accepted = True
        response.active_mode = self._active_mode
        response.message = message
        self._publish_transition(
            previous_mode=previous_mode,
            requested_mode=request.requested_mode,
            requester=requester,
            reason=reason,
            outcome=ModeTransitionState.OUTCOME_ACCEPTED,
            message=message,
            takeover=False,
        )
        self._publish_state()
        return response

    def _handle_takeover(self, request: Takeover.Request, response: Takeover.Response):
        requester = request.requester or "operator"
        reason = request.reason or "operator_takeover"
        previous_mode = self._active_mode
        accepted, message = self._evaluate_transition(ModeState.MODE_TELEOP, takeover=True)

        if accepted:
            self._apply_mode(ModeState.MODE_TELEOP, requester, reason)

        response.accepted = accepted
        response.previous_mode = previous_mode
        response.active_mode = self._active_mode
        response.message = message
        self._publish_transition(
            previous_mode=previous_mode,
            requested_mode=ModeState.MODE_TELEOP,
            requester=requester,
            reason=reason,
            outcome=(
                ModeTransitionState.OUTCOME_ACCEPTED
                if accepted
                else ModeTransitionState.OUTCOME_REJECTED
            ),
            message=message,
            takeover=True,
        )
        self._publish_state()
        return response

    def _on_safety_state(self, msg: SafetyState) -> None:
        self._safety_state = msg.state
        self._safety_seen = True

    def _evaluate_transition(self, requested_mode: int, takeover: bool) -> tuple[bool, str]:
        return evaluate_transition(
            active_mode=self._active_mode,
            requested_mode=requested_mode,
            safety_seen=self._safety_seen,
            safety_state=self._safety_state,
            takeover=takeover,
        )

    def _apply_mode(self, requested_mode: int, requester: str, reason: str) -> None:
        self._requested_mode = requested_mode
        self._active_mode = requested_mode
        self._owner = requester
        self._transition_reason = reason
        self._active_since = self.get_clock().now().to_msg()

    def _publish_state(self) -> None:
        robot_id = str(self.get_parameter("robot_id").value)
        msg = ModeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = robot_id
        msg.robot_id = robot_id
        msg.requested_mode = self._requested_mode
        msg.active_mode = self._active_mode
        msg.owner = self._owner
        msg.transition_reason = self._transition_reason
        msg.active_since = self._active_since
        self._mode_pub.publish(msg)

    def _publish_transition(
        self,
        previous_mode: int,
        requested_mode: int,
        requester: str,
        reason: str,
        outcome: int,
        message: str,
        takeover: bool,
    ) -> None:
        robot_id = str(self.get_parameter("robot_id").value)
        msg = ModeTransitionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = robot_id
        msg.robot_id = robot_id
        msg.previous_mode = previous_mode
        msg.requested_mode = requested_mode
        msg.active_mode = self._active_mode
        msg.requester = requester
        msg.reason = reason
        msg.outcome = outcome
        msg.message = message
        msg.takeover = takeover
        self._transition_pub.publish(msg)


def evaluate_transition(
    active_mode: int,
    requested_mode: int,
    safety_seen: bool,
    safety_state: int,
    takeover: bool,
) -> tuple[bool, str]:
    if active_mode == ModeState.MODE_SHUTDOWN and requested_mode != ModeState.MODE_SHUTDOWN:
        return False, "shutdown_is_terminal"

    if requested_mode == active_mode:
        return True, "mode_unchanged"

    if takeover:
        if active_mode not in ACTIVE_MODES:
            return False, "takeover_requires_autonomy_ai_or_teleop"
        return True, "operator_takeover_accepted"

    if requested_mode in ACTIVE_MODES and not safety_seen:
        return False, "safety_state_unknown"

    if requested_mode in AUTONOMY_MODES and safety_state in AUTONOMY_BLOCKING_SAFETY_STATES:
        return False, "safety_state_blocks_autonomy"

    if active_mode == ModeState.MODE_MAINTENANCE and requested_mode in AUTONOMY_MODES:
        return False, "maintenance_requires_intermediate_mode"

    if requested_mode == ModeState.MODE_AI_POLICY and active_mode not in AI_POLICY_ENTRY_MODES:
        return False, "ai_policy_requires_active_runtime_mode"

    return True, "mode accepted"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
