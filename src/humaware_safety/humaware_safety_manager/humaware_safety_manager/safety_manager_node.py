"""Initial safety manager node for HumaWare."""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from humaware_msgs.msg import ModeState, MRMState, SafetyState
from humaware_msgs.srv import ClearMRM, TriggerMRM


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
        watchdog_warnings, watchdog_mrm_reasons = self._evaluate_watchdogs(now)
        mrm_reason = self._select_mrm_reason(
            parameter_mrm_active=parameter_mrm_active,
            parameter_mrm_reason=parameter_mrm_reason,
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

    def _evaluate_watchdogs(self, now: Time) -> tuple[list[str], list[str]]:
        warnings: list[str] = []
        mrm_reasons: list[str] = []

        if self._active_mode == ModeState.MODE_TELEOP and bool(
            self.get_parameter("require_teleop_heartbeat").value
        ):
            self._check_heartbeat(
                now=now,
                name="teleop_heartbeat",
                last_seen=self._teleop_heartbeat_at,
                timeout_s=float(self.get_parameter("teleop_heartbeat_timeout_s").value),
                trigger_mrm=bool(
                    self.get_parameter("teleop_heartbeat_timeout_triggers_mrm").value
                ),
                warnings=warnings,
                mrm_reasons=mrm_reasons,
            )

        if self._active_mode in (
            ModeState.MODE_TELEOP,
            ModeState.MODE_AUTONOMY,
            ModeState.MODE_AI_POLICY,
        ) and bool(self.get_parameter("require_hardware_heartbeat").value):
            self._check_heartbeat(
                now=now,
                name="hardware_heartbeat",
                last_seen=self._hardware_heartbeat_at,
                timeout_s=float(self.get_parameter("hardware_heartbeat_timeout_s").value),
                trigger_mrm=bool(
                    self.get_parameter("hardware_heartbeat_timeout_triggers_mrm").value
                ),
                warnings=warnings,
                mrm_reasons=mrm_reasons,
            )

        if self._should_check_approved_command(now):
            reason = "approved_command_timeout"
            if bool(self.get_parameter("approved_command_timeout_triggers_mrm").value):
                mrm_reasons.append(reason)
            else:
                warnings.append(reason)

        return warnings, mrm_reasons

    def _check_heartbeat(
        self,
        now: Time,
        name: str,
        last_seen: Time | None,
        timeout_s: float,
        trigger_mrm: bool,
        warnings: list[str],
        mrm_reasons: list[str],
    ) -> None:
        reason = f"{name}_missing" if last_seen is None else f"{name}_timeout"
        timed_out = last_seen is None or now - last_seen > Duration(seconds=timeout_s)
        if not timed_out:
            return
        if trigger_mrm:
            mrm_reasons.append(reason)
        else:
            warnings.append(reason)

    def _should_check_approved_command(self, now: Time) -> bool:
        if not bool(self.get_parameter("monitor_approved_commands").value):
            return False
        if self._approved_command_at is None:
            return False
        if self._active_mode not in (
            ModeState.MODE_TELEOP,
            ModeState.MODE_AUTONOMY,
            ModeState.MODE_AI_POLICY,
        ):
            return False
        timeout_s = float(self.get_parameter("approved_command_timeout_s").value)
        return now - self._approved_command_at > Duration(seconds=timeout_s)

    def _select_mrm_reason(
        self,
        parameter_mrm_active: bool,
        parameter_mrm_reason: str,
        watchdog_mrm_reasons: list[str],
    ) -> str:
        if parameter_mrm_active and parameter_mrm_reason:
            return parameter_mrm_reason
        if self._service_mrm_active and self._service_mrm_reason:
            return self._service_mrm_reason
        if watchdog_mrm_reasons:
            return watchdog_mrm_reasons[0]
        if parameter_mrm_active:
            return "parameter_mrm_active"
        if self._service_mrm_active:
            return "service_mrm_active"
        return ""


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
