"""Runtime diagnostics aggregator for HumaWare."""

from dataclasses import dataclass
from typing import Any, Iterable

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from humaware_msgs.msg import (
    CommandArbitrationState,
    HealthState,
    LocomotionState,
    ModeState,
    ModeTransitionState,
    MRMState,
    NavigationBridgeState,
    SafetyState,
)


@dataclass
class TopicSample:
    """Last message and receive time for a monitored topic."""

    message: Any | None = None
    received_at: Time | None = None


def compute_stale_topics(
    samples: dict[str, TopicSample],
    required_topics: Iterable[str],
    now: Time,
    timeout: Duration,
) -> list[str]:
    """Return the subset of required topics whose last sample is stale or missing."""
    stale: list[str] = []
    for topic in required_topics:
        sample = samples.get(topic)
        if sample is None or sample.received_at is None:
            stale.append(topic)
            continue
        if now - sample.received_at > timeout:
            stale.append(topic)
    return stale


def evaluate_health(
    safety_state: int,
    locomotion_state: int,
    active_faults: list[str],
    active_warnings: list[str],
    stale_topics: list[str],
) -> tuple[int, str]:
    """Map runtime state into a HealthState value and a short summary."""
    if stale_topics:
        return HealthState.HEALTH_STALE, "stale runtime topics"

    if safety_state in (
        SafetyState.STATE_FAULT,
        SafetyState.STATE_ESTOP,
        SafetyState.STATE_MRM,
    ):
        return HealthState.HEALTH_ERROR, "safety state requires intervention"

    if locomotion_state == LocomotionState.STATE_FAULT:
        return HealthState.HEALTH_ERROR, "locomotion fault"

    if active_faults:
        return HealthState.HEALTH_ERROR, "active faults"

    if safety_state == SafetyState.STATE_WARN or active_warnings:
        return HealthState.HEALTH_WARN, "active warnings"

    return HealthState.HEALTH_OK, "runtime healthy"


class DiagnosticsAggregatorNode(Node):
    """Aggregate runtime topics into standard diagnostics and health state."""

    def __init__(self) -> None:
        super().__init__("diagnostics_aggregator")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("stale_timeout_s", 1.5)
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("monitor_nav2_bridge", False)
        self.declare_parameter("monitor_teleop_heartbeat", False)

        self._samples: dict[str, TopicSample] = {
            "mode/state": TopicSample(),
            "mode/transition_state": TopicSample(),
            "safety/state": TopicSample(),
            "safety/mrm_state": TopicSample(),
            "locomotion/state": TopicSample(),
            "runtime/command_arbitration_state": TopicSample(),
            "navigation/nav2_bridge_state": TopicSample(),
            "teleop/heartbeat": TopicSample(),
        }

        self._health_pub = self.create_publisher(HealthState, "runtime/health", 10)
        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)

        self._subscribe("mode/state", ModeState)
        self._subscribe("mode/transition_state", ModeTransitionState)
        self._subscribe("safety/state", SafetyState)
        self._subscribe("safety/mrm_state", MRMState)
        self._subscribe("locomotion/state", LocomotionState)
        self._subscribe("runtime/command_arbitration_state", CommandArbitrationState)
        self._subscribe("navigation/nav2_bridge_state", NavigationBridgeState)
        self._subscribe("teleop/heartbeat", Header)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._publish)

    def _subscribe(self, topic: str, msg_type: type) -> None:
        self.create_subscription(
            msg_type,
            topic,
            lambda msg, topic=topic: self._on_topic(topic, msg),
            10,
        )

    def _on_topic(self, topic: str, msg: Any) -> None:
        self._samples[topic] = TopicSample(message=msg, received_at=self.get_clock().now())

    def _publish(self) -> None:
        now = self.get_clock().now()
        stale_topics = self._stale_topics(now)
        health = self._build_health(now, stale_topics)
        diagnostics = self._build_diagnostics(now, health, stale_topics)
        self._health_pub.publish(health)
        self._diagnostics_pub.publish(diagnostics)

    def _required_topics(self) -> list[str]:
        required = [
            "mode/state",
            "safety/state",
            "safety/mrm_state",
            "locomotion/state",
            "runtime/command_arbitration_state",
        ]
        if bool(self.get_parameter("monitor_nav2_bridge").value):
            required.append("navigation/nav2_bridge_state")
        if bool(self.get_parameter("monitor_teleop_heartbeat").value):
            required.append("teleop/heartbeat")
        return required

    def _stale_topics(self, now: Time) -> list[str]:
        timeout = Duration(seconds=float(self.get_parameter("stale_timeout_s").value))
        return compute_stale_topics(
            samples=self._samples,
            required_topics=self._required_topics(),
            now=now,
            timeout=timeout,
        )

    def _build_health(self, now: Time, stale_topics: list[str]) -> HealthState:
        robot_id = str(self.get_parameter("robot_id").value)
        mode = self._message("mode/state", ModeState)
        safety = self._message("safety/state", SafetyState)
        locomotion = self._message("locomotion/state", LocomotionState)
        command = self._message("runtime/command_arbitration_state", CommandArbitrationState)
        nav2 = self._message("navigation/nav2_bridge_state", NavigationBridgeState)

        health = HealthState()
        health.header.stamp = now.to_msg()
        health.header.frame_id = robot_id
        health.robot_id = robot_id
        health.mode = mode.active_mode if mode is not None else ModeState.MODE_UNKNOWN
        health.safety_state = safety.state if safety is not None else SafetyState.STATE_UNKNOWN
        health.locomotion_state = (
            locomotion.state if locomotion is not None else LocomotionState.STATE_UNKNOWN
        )
        health.command_source = (
            command.active_source if command is not None else CommandArbitrationState.SOURCE_NONE
        )
        health.nav2_bridge_state = (
            nav2.state if nav2 is not None else NavigationBridgeState.STATE_UNKNOWN
        )
        health.command_output_enabled = bool(command.output_enabled) if command is not None else False
        health.nav2_output_enabled = bool(nav2.output_enabled) if nav2 is not None else False
        health.active_faults = list(safety.active_faults) if safety is not None else []
        health.active_warnings = list(safety.active_warnings) if safety is not None else []
        health.stale_topics = stale_topics
        health.state, health.summary = evaluate_health(
            safety_state=health.safety_state,
            locomotion_state=health.locomotion_state,
            active_faults=list(health.active_faults),
            active_warnings=list(health.active_warnings),
            stale_topics=list(health.stale_topics),
        )
        return health

    def _build_diagnostics(
        self,
        now: Time,
        health: HealthState,
        stale_topics: list[str],
    ) -> DiagnosticArray:
        array = DiagnosticArray()
        array.header.stamp = now.to_msg()
        array.status = [
            self._runtime_status(health),
            self._topic_status("mode/state", stale_topics),
            self._topic_status("safety/state", stale_topics),
            self._topic_status("locomotion/state", stale_topics),
            self._topic_status("runtime/command_arbitration_state", stale_topics),
        ]

        if bool(self.get_parameter("monitor_nav2_bridge").value) or self._has_seen(
            "navigation/nav2_bridge_state"
        ):
            array.status.append(self._topic_status("navigation/nav2_bridge_state", stale_topics))

        if bool(self.get_parameter("monitor_teleop_heartbeat").value) or self._has_seen(
            "teleop/heartbeat"
        ):
            array.status.append(self._topic_status("teleop/heartbeat", stale_topics))

        return array

    def _runtime_status(self, health: HealthState) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f"{health.robot_id}/runtime_health"
        status.hardware_id = health.robot_id
        status.level = self._diagnostic_level(health.state)
        status.message = health.summary
        status.values = [
            self._kv("mode", health.mode),
            self._kv("safety_state", health.safety_state),
            self._kv("locomotion_state", health.locomotion_state),
            self._kv("command_source", health.command_source),
            self._kv("nav2_bridge_state", health.nav2_bridge_state),
            self._kv("command_output_enabled", health.command_output_enabled),
            self._kv("nav2_output_enabled", health.nav2_output_enabled),
            self._kv("stale_topics", ",".join(health.stale_topics)),
            self._kv("active_warnings", ",".join(health.active_warnings)),
            self._kv("active_faults", ",".join(health.active_faults)),
        ]
        return status

    def _topic_status(self, topic: str, stale_topics: list[str]) -> DiagnosticStatus:
        robot_id = str(self.get_parameter("robot_id").value)
        sample = self._samples[topic]
        status = DiagnosticStatus()
        status.name = f"{robot_id}/{topic}"
        status.hardware_id = robot_id
        if topic in stale_topics:
            status.level = DiagnosticStatus.STALE
            status.message = "stale" if sample.received_at is not None else "missing"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"
        status.values = [
            self._kv("topic", topic),
            self._kv("seen", sample.received_at is not None),
        ]
        return status

    def _message(self, topic: str, expected_type: type) -> Any | None:
        msg = self._samples[topic].message
        if isinstance(msg, expected_type):
            return msg
        return None

    def _has_seen(self, topic: str) -> bool:
        return self._samples[topic].received_at is not None

    @staticmethod
    def _diagnostic_level(health_state: int):
        if health_state == HealthState.HEALTH_OK:
            return DiagnosticStatus.OK
        if health_state == HealthState.HEALTH_WARN:
            return DiagnosticStatus.WARN
        if health_state == HealthState.HEALTH_ERROR:
            return DiagnosticStatus.ERROR
        if health_state == HealthState.HEALTH_STALE:
            return DiagnosticStatus.STALE
        return DiagnosticStatus.STALE

    @staticmethod
    def _kv(key: str, value: Any) -> KeyValue:
        return KeyValue(key=key, value=str(value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiagnosticsAggregatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
