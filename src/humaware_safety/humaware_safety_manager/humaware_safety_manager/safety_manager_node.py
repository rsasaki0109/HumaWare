"""Initial safety manager node for HumaWare."""

from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from humaware_msgs.msg import ModeState, MRMState, SafetyState
from humaware_msgs.srv import ClearMRM, TriggerMRM


HEARTBEAT_MODES = (ModeState.MODE_TELEOP, ModeState.MODE_AUTONOMY, ModeState.MODE_AI_POLICY)


@dataclass
class HeartbeatPolicy:
    """Configuration for a single heartbeat watchdog."""

    name: str
    timeout: Duration
    trigger_mrm: bool


@dataclass
class WatchdogResult:
    """Output of the watchdog evaluation."""

    warnings: List[str] = field(default_factory=list)
    mrm_reasons: List[str] = field(default_factory=list)


def check_heartbeat(
    now: Time,
    last_seen: Optional[Time],
    policy: HeartbeatPolicy,
) -> Optional[tuple[str, bool]]:
    """Return (reason, triggers_mrm) when the heartbeat is missing or stale."""
    if last_seen is None:
        return f"{policy.name}_missing", policy.trigger_mrm
    if now - last_seen > policy.timeout:
        return f"{policy.name}_timeout", policy.trigger_mrm
    return None


def evaluate_watchdogs(
    now: Time,
    active_mode: int,
    teleop_last_seen: Optional[Time],
    hardware_last_seen: Optional[Time],
    approved_last_seen: Optional[Time],
    require_teleop_heartbeat: bool,
    teleop_policy: HeartbeatPolicy,
    require_hardware_heartbeat: bool,
    hardware_policy: HeartbeatPolicy,
    monitor_approved_commands: bool,
    approved_command_timeout: Duration,
    approved_command_timeout_triggers_mrm: bool,
) -> WatchdogResult:
    """Return warnings and MRM reasons derived from watchdog state."""
    result = WatchdogResult()

    if active_mode == ModeState.MODE_TELEOP and require_teleop_heartbeat:
        finding = check_heartbeat(now, teleop_last_seen, teleop_policy)
        if finding is not None:
            reason, triggers_mrm = finding
            (result.mrm_reasons if triggers_mrm else result.warnings).append(reason)

    if active_mode in HEARTBEAT_MODES and require_hardware_heartbeat:
        finding = check_heartbeat(now, hardware_last_seen, hardware_policy)
        if finding is not None:
            reason, triggers_mrm = finding
            (result.mrm_reasons if triggers_mrm else result.warnings).append(reason)

    # The approved-command watchdog is a *staleness* detector, deliberately
    # asymmetric with the heartbeat *presence* detectors above (which fire
    # `_missing` the instant last_seen is None). Heartbeats are produced
    # continuously from boot regardless of mode, so a None there means the
    # publisher is absent -- a real fault. Approved commands, by contrast,
    # only exist once the arbiter starts emitting in an active mode, so a None
    # here is the *normal* boot / mode-entry transient, not a fault. Firing on
    # None would raise a spurious warning -- or, under
    # approved_command_timeout_triggers_mrm, a spurious MRM that blocks startup
    # at the worst moment. The genuinely dangerous case (a dead arbiter that
    # never produces approved commands) is already caught downstream: both the
    # locomotion adapter and the hardware adapter gate age out their approved
    # input and stop / MRM the robot. So this watchdog only measures staleness
    # against an established baseline.
    if (
        monitor_approved_commands
        and approved_last_seen is not None
        and active_mode in HEARTBEAT_MODES
        and now - approved_last_seen > approved_command_timeout
    ):
        reason = "approved_command_timeout"
        if approved_command_timeout_triggers_mrm:
            result.mrm_reasons.append(reason)
        else:
            result.warnings.append(reason)

    return result


def select_mrm_reason(
    parameter_mrm_active: bool,
    parameter_mrm_reason: str,
    service_mrm_active: bool,
    service_mrm_reason: str,
    watchdog_mrm_reasons: List[str],
) -> str:
    """Return the textual reason for an active MRM, or empty when not active."""
    if parameter_mrm_active and parameter_mrm_reason:
        return parameter_mrm_reason
    if service_mrm_active and service_mrm_reason:
        return service_mrm_reason
    if watchdog_mrm_reasons:
        return watchdog_mrm_reasons[0]
    if parameter_mrm_active:
        return "parameter_mrm_active"
    if service_mrm_active:
        return "service_mrm_active"
    return ""


class SafetyManagerNode(Node):
    """Publish safety and MRM state for the initial runtime boundary."""

    def __init__(self) -> None:
        super().__init__("safety_manager")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("estop_engaged", False)
        self.declare_parameter("mrm_active", False)
        self.declare_parameter("mrm_reason", "")
        self.declare_parameter("require_teleop_heartbeat", True)
        self.declare_parameter("teleop_heartbeat_timeout_s", 1.0)
        self.declare_parameter("teleop_heartbeat_timeout_triggers_mrm", False)
        self.declare_parameter("require_hardware_heartbeat", False)
        self.declare_parameter("hardware_heartbeat_timeout_s", 1.0)
        self.declare_parameter("hardware_heartbeat_timeout_triggers_mrm", True)
        self.declare_parameter("monitor_approved_commands", True)
        self.declare_parameter("approved_command_timeout_s", 1.0)
        self.declare_parameter("approved_command_timeout_triggers_mrm", False)

        self._safety_pub = self.create_publisher(SafetyState, "safety/state", 10)
        self._mrm_pub = self.create_publisher(MRMState, "safety/mrm_state", 10)
        self._trigger_mrm_srv = self.create_service(
            TriggerMRM,
            "safety/trigger_mrm",
            self._handle_trigger_mrm,
        )
        self._clear_mrm_srv = self.create_service(
            ClearMRM,
            "safety/clear_mrm",
            self._handle_clear_mrm,
        )

        self.create_subscription(Header, "teleop/heartbeat", self._on_teleop_heartbeat, 10)
        self.create_subscription(Header, "hardware/heartbeat", self._on_hardware_heartbeat, 10)
        self.create_subscription(TwistStamped, "cmd_vel/approved", self._on_approved_command, 10)
        self.create_subscription(ModeState, "mode/state", self._on_mode_state, 10)

        self._active_mode = ModeState.MODE_INACTIVE
        self._teleop_heartbeat_at: Time | None = None
        self._hardware_heartbeat_at: Time | None = None
        self._approved_command_at: Time | None = None
        self._service_mrm_active = False
        self._service_mrm_reason = ""
        self._current_safety_state = SafetyState.STATE_UNKNOWN

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._publish_state)

    def _on_teleop_heartbeat(self, _msg: Header) -> None:
        self._teleop_heartbeat_at = self.get_clock().now()

    def _on_hardware_heartbeat(self, _msg: Header) -> None:
        self._hardware_heartbeat_at = self.get_clock().now()

    def _on_approved_command(self, _msg: TwistStamped) -> None:
        self._approved_command_at = self.get_clock().now()

    def _on_mode_state(self, msg: ModeState) -> None:
        self._active_mode = msg.active_mode

    def _handle_trigger_mrm(
        self,
        request: TriggerMRM.Request,
        response: TriggerMRM.Response,
    ):
        self._service_mrm_active = True
        self._service_mrm_reason = request.reason or "manual_mrm_trigger"
        self._publish_state()
        response.accepted = True
        response.active_safety_state = self._current_safety_state
        requester = request.requester or "anonymous"
        response.message = f"MRM triggered by {requester}: {self._service_mrm_reason}"
        return response

    def _handle_clear_mrm(
        self,
        request: ClearMRM.Request,
        response: ClearMRM.Response,
    ):
        if bool(self.get_parameter("estop_engaged").value):
            response.accepted = False
            response.active_safety_state = SafetyState.STATE_ESTOP
            response.message = "cannot clear MRM while E-stop is engaged"
            return response

        self._service_mrm_active = False
        self._service_mrm_reason = ""
        self._publish_state()
        response.accepted = True
        response.active_safety_state = self._current_safety_state
        requester = request.requester or "anonymous"
        reason = request.reason or "manual_mrm_clear"
        response.message = f"MRM cleared by {requester}: {reason}"
        return response

    def _publish_state(self) -> None:
        now = self.get_clock().now()
        robot_id = str(self.get_parameter("robot_id").value)
        estop_engaged = bool(self.get_parameter("estop_engaged").value)
        parameter_mrm_active = bool(self.get_parameter("mrm_active").value)
        parameter_mrm_reason = str(self.get_parameter("mrm_reason").value)
        watchdogs = evaluate_watchdogs(
            now=now,
            active_mode=self._active_mode,
            teleop_last_seen=self._teleop_heartbeat_at,
            hardware_last_seen=self._hardware_heartbeat_at,
            approved_last_seen=self._approved_command_at,
            require_teleop_heartbeat=bool(self.get_parameter("require_teleop_heartbeat").value),
            teleop_policy=HeartbeatPolicy(
                name="teleop_heartbeat",
                timeout=Duration(
                    seconds=float(self.get_parameter("teleop_heartbeat_timeout_s").value)
                ),
                trigger_mrm=bool(
                    self.get_parameter("teleop_heartbeat_timeout_triggers_mrm").value
                ),
            ),
            require_hardware_heartbeat=bool(
                self.get_parameter("require_hardware_heartbeat").value
            ),
            hardware_policy=HeartbeatPolicy(
                name="hardware_heartbeat",
                timeout=Duration(
                    seconds=float(self.get_parameter("hardware_heartbeat_timeout_s").value)
                ),
                trigger_mrm=bool(
                    self.get_parameter("hardware_heartbeat_timeout_triggers_mrm").value
                ),
            ),
            monitor_approved_commands=bool(
                self.get_parameter("monitor_approved_commands").value
            ),
            approved_command_timeout=Duration(
                seconds=float(self.get_parameter("approved_command_timeout_s").value)
            ),
            approved_command_timeout_triggers_mrm=bool(
                self.get_parameter("approved_command_timeout_triggers_mrm").value
            ),
        )
        watchdog_warnings = watchdogs.warnings
        watchdog_mrm_reasons = watchdogs.mrm_reasons
        mrm_reason = select_mrm_reason(
            parameter_mrm_active=parameter_mrm_active,
            parameter_mrm_reason=parameter_mrm_reason,
            service_mrm_active=self._service_mrm_active,
            service_mrm_reason=self._service_mrm_reason,
            watchdog_mrm_reasons=watchdog_mrm_reasons,
        )
        mrm_active = parameter_mrm_active or self._service_mrm_active or bool(watchdog_mrm_reasons)

        safety = SafetyState()
        safety.header.stamp = now.to_msg()
        safety.header.frame_id = robot_id
        safety.robot_id = robot_id
        safety.estop_engaged = estop_engaged
        safety.mrm_active = mrm_active
        safety.last_heartbeat = now.to_msg()

        if estop_engaged:
            safety.state = SafetyState.STATE_ESTOP
            safety.active_faults = ["estop_engaged"]
        elif mrm_active:
            safety.state = SafetyState.STATE_MRM
            safety.active_warnings = ["minimal_risk_maneuver_active"] + watchdog_warnings
            if mrm_reason:
                safety.active_warnings.append(mrm_reason)
        elif watchdog_warnings:
            safety.state = SafetyState.STATE_WARN
            safety.active_warnings = watchdog_warnings
        else:
            safety.state = SafetyState.STATE_OK
        self._current_safety_state = safety.state

        mrm = MRMState()
        mrm.header.stamp = now.to_msg()
        mrm.header.frame_id = robot_id
        mrm.robot_id = robot_id
        mrm.reason = mrm_reason
        mrm.timeout = Duration(seconds=0.0).to_msg()
        mrm.operator_intervention_requested = mrm_active or estop_engaged

        if estop_engaged:
            mrm.state = MRMState.STATE_FAULT
        elif mrm_active:
            mrm.state = MRMState.STATE_STOP
        else:
            mrm.state = MRMState.STATE_NONE

        self._safety_pub.publish(safety)
        self._mrm_pub.publish(mrm)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
