"""Runtime capability registry for HumaWare."""

from dataclasses import dataclass
from typing import Iterable

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from humaware_msgs.msg import Capability, CapabilityRegistry, LocomotionState, ModeState, SafetyState
from humaware_msgs.srv import ListCapabilities


@dataclass(frozen=True)
class CapabilitySpec:
    """Static capability metadata."""

    name: str
    required_mode: str
    owner_node: str
    required_hardware: tuple[str, ...]
    input_type: str
    output_type: str
    timeout_s: float
    safety_constraints: tuple[str, ...]
    recovery_behavior: str


DEFAULT_CAPABILITIES = (
    CapabilitySpec(
        name="stand",
        required_mode="inactive|teleop|autonomy|ai_policy",
        owner_node="mock_locomotion_adapter",
        required_hardware=("legs", "imu"),
        input_type="std_msgs/msg/Empty",
        output_type="humaware_msgs/msg/LocomotionState",
        timeout_s=5.0,
        safety_constraints=("safety_ok_or_warn", "not_estop", "balance_required"),
        recovery_behavior="stop",
    ),
    CapabilitySpec(
        name="stop",
        required_mode="any",
        owner_node="command_arbiter",
        required_hardware=("legs",),
        input_type="std_msgs/msg/Empty",
        output_type="geometry_msgs/msg/TwistStamped",
        timeout_s=0.5,
        safety_constraints=("always_available", "command_timeout"),
        recovery_behavior="hold_stop",
    ),
    CapabilitySpec(
        name="walk_velocity",
        required_mode="teleop|autonomy|ai_policy",
        owner_node="command_arbiter",
        required_hardware=("legs", "imu"),
        input_type="geometry_msgs/msg/TwistStamped",
        output_type="humaware_msgs/msg/LocomotionState",
        timeout_s=0.75,
        safety_constraints=("safety_ok_or_warn", "velocity_limit", "balance_required"),
        recovery_behavior="stop",
    ),
    CapabilitySpec(
        name="turn_in_place",
        required_mode="teleop|autonomy|ai_policy",
        owner_node="command_arbiter",
        required_hardware=("legs", "imu"),
        input_type="geometry_msgs/msg/TwistStamped",
        output_type="humaware_msgs/msg/LocomotionState",
        timeout_s=0.75,
        safety_constraints=("safety_ok_or_warn", "angular_velocity_limit", "balance_required"),
        recovery_behavior="stop",
    ),
    CapabilitySpec(
        name="recover_posture",
        required_mode="maintenance|inactive|teleop",
        owner_node="safety_manager",
        required_hardware=("legs", "imu"),
        input_type="std_msgs/msg/Empty",
        output_type="humaware_msgs/msg/SafetyState",
        timeout_s=15.0,
        safety_constraints=("operator_supervision", "not_estop"),
        recovery_behavior="request_teleop",
    ),
    CapabilitySpec(
        name="request_teleop",
        required_mode="any",
        owner_node="mode_manager",
        required_hardware=(),
        input_type="std_msgs/msg/String",
        output_type="humaware_msgs/msg/ModeState",
        timeout_s=2.0,
        safety_constraints=("operator_available",),
        recovery_behavior="hold_mode",
    ),
)


class CapabilityRegistryNode(Node):
    """Publish and serve the current capability registry."""

    def __init__(self) -> None:
        super().__init__("capability_registry")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("publish_rate_hz", 1.0)

        self._mode = ModeState.MODE_UNKNOWN
        self._safety = SafetyState.STATE_UNKNOWN
        self._locomotion = LocomotionState.STATE_UNKNOWN

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._publisher = self.create_publisher(CapabilityRegistry, "capabilities", qos)
        self.create_service(ListCapabilities, "capabilities/list", self._on_list_capabilities)

        self.create_subscription(ModeState, "mode/state", self._on_mode_state, 10)
        self.create_subscription(SafetyState, "safety/state", self._on_safety_state, 10)
        self.create_subscription(LocomotionState, "locomotion/state", self._on_locomotion_state, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(publish_rate_hz, 0.1), self._publish)
        self._publish()

    def _on_mode_state(self, msg: ModeState) -> None:
        self._mode = msg.active_mode

    def _on_safety_state(self, msg: SafetyState) -> None:
        self._safety = msg.state

    def _on_locomotion_state(self, msg: LocomotionState) -> None:
        self._locomotion = msg.state

    def _on_list_capabilities(self, request, response):
        capabilities = self._capabilities()
        by_name = {capability.name: capability for capability in capabilities}
        requested_names = list(request.names)
        if requested_names:
            response.capabilities = [by_name[name] for name in requested_names if name in by_name]
            response.missing_names = [name for name in requested_names if name not in by_name]
        else:
            response.capabilities = capabilities
            response.missing_names = []
        return response

    def _publish(self) -> None:
        registry = CapabilityRegistry()
        registry.header.stamp = self.get_clock().now().to_msg()
        registry.header.frame_id = self.robot_id
        registry.robot_id = self.robot_id
        registry.capabilities = self._capabilities()
        self._publisher.publish(registry)

    @property
    def robot_id(self) -> str:
        return str(self.get_parameter("robot_id").value)

    def _capabilities(self) -> list[Capability]:
        return [
            build_capability(spec, self._mode, self._safety, self._locomotion)
            for spec in DEFAULT_CAPABILITIES
        ]


def build_capability(
    spec: CapabilitySpec,
    mode: int,
    safety: int,
    locomotion: int,
) -> Capability:
    capability = Capability()
    capability.name = spec.name
    capability.state = capability_state(spec, mode, safety, locomotion)
    capability.required_mode = spec.required_mode
    capability.owner_node = spec.owner_node
    capability.required_hardware = list(spec.required_hardware)
    capability.input_type = spec.input_type
    capability.output_type = spec.output_type
    capability.timeout = duration_msg(spec.timeout_s)
    capability.safety_constraints = list(spec.safety_constraints)
    capability.recovery_behavior = spec.recovery_behavior
    return capability


def capability_state(spec: CapabilitySpec, mode: int, safety: int, locomotion: int) -> int:
    if mode == ModeState.MODE_SHUTDOWN:
        return Capability.STATE_UNAVAILABLE

    if spec.name == "stop":
        return Capability.STATE_IDLE

    if spec.name == "request_teleop":
        return Capability.STATE_DEGRADED if safety in intervention_states() else Capability.STATE_IDLE

    if safety in (SafetyState.STATE_FAULT, SafetyState.STATE_ESTOP):
        return Capability.STATE_FAULT

    if safety == SafetyState.STATE_MRM:
        return Capability.STATE_DEGRADED if spec.name == "recover_posture" else Capability.STATE_UNAVAILABLE

    if safety == SafetyState.STATE_UNKNOWN:
        return Capability.STATE_UNAVAILABLE

    if spec.name in {"walk_velocity", "turn_in_place"} and mode not in active_motion_modes():
        return Capability.STATE_UNAVAILABLE

    if spec.name == "recover_posture" and mode not in recover_modes():
        return Capability.STATE_UNAVAILABLE

    if spec.name == "stand" and mode == ModeState.MODE_MAINTENANCE:
        return Capability.STATE_UNAVAILABLE

    if spec.name in {"walk_velocity", "turn_in_place"} and locomotion in executing_locomotion_states():
        return Capability.STATE_EXECUTING

    return Capability.STATE_IDLE if safety == SafetyState.STATE_OK else Capability.STATE_DEGRADED


def active_motion_modes() -> set[int]:
    return {ModeState.MODE_TELEOP, ModeState.MODE_AUTONOMY, ModeState.MODE_AI_POLICY}


def recover_modes() -> set[int]:
    return {ModeState.MODE_MAINTENANCE, ModeState.MODE_INACTIVE, ModeState.MODE_TELEOP}


def executing_locomotion_states() -> set[int]:
    return {LocomotionState.STATE_WALKING, LocomotionState.STATE_TURNING}


def intervention_states() -> set[int]:
    return {SafetyState.STATE_FAULT, SafetyState.STATE_ESTOP, SafetyState.STATE_MRM}


def duration_msg(seconds: float) -> DurationMsg:
    msg = DurationMsg()
    whole_seconds = int(seconds)
    msg.sec = whole_seconds
    msg.nanosec = int((seconds - whole_seconds) * 1_000_000_000)
    return msg


def capability_names(capabilities: Iterable[Capability]) -> set[str]:
    return {capability.name for capability in capabilities}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CapabilityRegistryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
