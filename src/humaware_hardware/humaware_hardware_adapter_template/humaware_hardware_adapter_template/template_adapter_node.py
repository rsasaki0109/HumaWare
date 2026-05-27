"""Template hardware adapter node.

This module is a starting point for vendor-specific HumaWare hardware
adapters. It implements the runtime contract every adapter must honor but
intentionally does not translate approved commands into vendor commands.

Adapter authors should copy this package, rename it, and replace the
``_apply_to_hardware`` stub with vendor command translation. The runtime
must not regress: hardware output must remain gated on the approved
command topic, safety state, and runtime mode.

The defaults here also make it safe to run the template in CI: no vendor
SDK is touched and no actuator topic is published.
"""

from dataclasses import dataclass
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from humaware_msgs.msg import (
    CommandArbitrationState,
    ModeState,
    MRMState,
    SafetyState,
)


GATE_OPEN_REASON = "approved_command"


@dataclass
class AdapterIdentity:
    """Identity metadata published with adapter diagnostics."""

    robot_model: str
    firmware_version: str
    sdk_version: str
    git_sha: str
    launch_profile: str

    def as_key_values(self) -> list[KeyValue]:
        return [
            KeyValue(key="robot_model", value=self.robot_model),
            KeyValue(key="firmware_version", value=self.firmware_version),
            KeyValue(key="sdk_version", value=self.sdk_version),
            KeyValue(key="git_sha", value=self.git_sha),
            KeyValue(key="launch_profile", value=self.launch_profile),
        ]


def should_release_output(
    safety_state: int,
    mrm_state: int,
    active_mode: int,
    last_approved_at: Optional[Time],
    last_arbitration_at: Optional[Time],
    arbitration_output_enabled: bool,
    now: Time,
    approved_command_timeout: Duration,
    arbitration_state_timeout: Duration,
) -> tuple[bool, str]:
    """Return (allow_output, reason) for the adapter command gate.

    The adapter must keep the gate closed whenever the runtime is unsafe
    or the approved command stream is stale. The function is pure to keep
    these rules testable.
    """
    if safety_state in (
        SafetyState.STATE_FAULT,
        SafetyState.STATE_ESTOP,
        SafetyState.STATE_MRM,
    ):
        return False, "safety_state_blocks_output"
    if safety_state not in (SafetyState.STATE_OK, SafetyState.STATE_WARN):
        return False, "safety_state_not_ready"
    if mrm_state != MRMState.STATE_NONE:
        return False, "mrm_state_blocks_output"
    if active_mode not in (
        ModeState.MODE_TELEOP,
        ModeState.MODE_AUTONOMY,
        ModeState.MODE_AI_POLICY,
    ):
        return False, "active_mode_blocks_output"
    if last_arbitration_at is None:
        return False, "waiting_for_arbitration_state"
    if now - last_arbitration_at > arbitration_state_timeout:
        return False, "arbitration_state_stale"
    if not arbitration_output_enabled:
        return False, "arbiter_disabled_output"
    if last_approved_at is None:
        return False, "waiting_for_approved_command"
    if now - last_approved_at > approved_command_timeout:
        return False, "approved_command_stale"
    return True, GATE_OPEN_REASON


class HardwareAdapterTemplateNode(Node):
    """Stub hardware adapter that respects the HumaWare runtime contract."""

    def __init__(self) -> None:
        super().__init__("hardware_adapter_template")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("robot_model", "unset")
        self.declare_parameter("firmware_version", "unset")
        self.declare_parameter("sdk_version", "unset")
        self.declare_parameter("git_sha", "unset")
        self.declare_parameter("launch_profile", "unset")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("heartbeat_rate_hz", 10.0)
        self.declare_parameter("approved_command_timeout_s", 0.5)
        self.declare_parameter("arbitration_state_timeout_s", 1.0)

        self._safety_state = SafetyState.STATE_UNKNOWN
        self._mrm_state = MRMState.STATE_NONE
        self._active_mode = ModeState.MODE_INACTIVE
        self._last_approved: Optional[TwistStamped] = None
        self._last_approved_at: Optional[Time] = None
        self._last_arbitration: Optional[CommandArbitrationState] = None
        self._last_arbitration_at: Optional[Time] = None

        self._heartbeat_pub = self.create_publisher(Header, "hardware/heartbeat", 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_subscription(SafetyState, "safety/state", self._on_safety_state, 10)
        self.create_subscription(MRMState, "safety/mrm_state", self._on_mrm_state, 10)
        self.create_subscription(ModeState, "mode/state", self._on_mode_state, 10)
        self.create_subscription(
            TwistStamped,
            "cmd_vel/approved",
            self._on_approved_command,
            10,
        )
        self.create_subscription(
            CommandArbitrationState,
            "runtime/command_arbitration_state",
            self._on_arbitration_state,
            10,
        )

        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(period, self._tick)
        heartbeat_period = 1.0 / max(float(self.get_parameter("heartbeat_rate_hz").value), 0.1)
        self.create_timer(heartbeat_period, self._publish_heartbeat)

    def _on_safety_state(self, msg: SafetyState) -> None:
        self._safety_state = msg.state

    def _on_mrm_state(self, msg: MRMState) -> None:
        self._mrm_state = msg.state

    def _on_mode_state(self, msg: ModeState) -> None:
        self._active_mode = msg.active_mode

    def _on_approved_command(self, msg: TwistStamped) -> None:
        self._last_approved = msg
        self._last_approved_at = self.get_clock().now()

    def _on_arbitration_state(self, msg: CommandArbitrationState) -> None:
        self._last_arbitration = msg
        self._last_arbitration_at = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        allow, reason = should_release_output(
            safety_state=self._safety_state,
            mrm_state=self._mrm_state,
            active_mode=self._active_mode,
            last_approved_at=self._last_approved_at,
            last_arbitration_at=self._last_arbitration_at,
            arbitration_output_enabled=bool(
                self._last_arbitration is not None and self._last_arbitration.output_enabled
            ),
            now=now,
            approved_command_timeout=Duration(
                seconds=float(self.get_parameter("approved_command_timeout_s").value)
            ),
            arbitration_state_timeout=Duration(
                seconds=float(self.get_parameter("arbitration_state_timeout_s").value)
            ),
        )

        self._publish_diagnostics(allow=allow, reason=reason, now=now)

        if not allow or self._last_approved is None:
            self._stop_hardware(reason)
            return

        self._apply_to_hardware(self._last_approved, reason)

    def _publish_heartbeat(self) -> None:
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = str(self.get_parameter("robot_id").value)
        self._heartbeat_pub.publish(msg)

    def _publish_diagnostics(self, allow: bool, reason: str, now: Time) -> None:
        identity = self._identity()
        status = DiagnosticStatus()
        status.name = f"{self.get_parameter('robot_id').value}/hardware_adapter_template"
        status.hardware_id = identity.robot_model
        status.level = DiagnosticStatus.OK if allow else DiagnosticStatus.WARN
        status.message = reason
        status.values = identity.as_key_values() + [
            KeyValue(key="output_allowed", value=str(allow).lower()),
            KeyValue(key="active_mode", value=str(self._active_mode)),
            KeyValue(key="safety_state", value=str(self._safety_state)),
            KeyValue(key="mrm_state", value=str(self._mrm_state)),
        ]
        array = DiagnosticArray()
        array.header.stamp = now.to_msg()
        array.status.append(status)
        self._diagnostics_pub.publish(array)

    def _identity(self) -> AdapterIdentity:
        return AdapterIdentity(
            robot_model=str(self.get_parameter("robot_model").value),
            firmware_version=str(self.get_parameter("firmware_version").value),
            sdk_version=str(self.get_parameter("sdk_version").value),
            git_sha=str(self.get_parameter("git_sha").value),
            launch_profile=str(self.get_parameter("launch_profile").value),
        )

    def _apply_to_hardware(self, command: TwistStamped, reason: str) -> None:
        """Translate an approved command into vendor commands.

        Adapter authors must replace this stub with vendor SDK calls. The
        template intentionally performs no actuation so the package is
        safe to run in CI.
        """
        self.get_logger().debug(
            f"approved command would be applied: linear.x={command.twist.linear.x:.3f}"
            f" angular.z={command.twist.angular.z:.3f} reason={reason}"
        )

    def _stop_hardware(self, reason: str) -> None:
        """Bring the vendor command stream to a safe stop.

        Adapter authors must replace this stub with vendor-specific stop
        behavior (zero velocity, brake engagement, posture hold, etc.).
        """
        self.get_logger().debug(f"adapter gate closed: {reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareAdapterTemplateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
