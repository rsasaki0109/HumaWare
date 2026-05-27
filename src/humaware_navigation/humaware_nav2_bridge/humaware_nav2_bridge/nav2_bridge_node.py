"""Nav2-style velocity bridge for HumaWare."""

from typing import Optional

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from humaware_msgs.msg import (
    LocomotionState,
    ModeState,
    NavigationBridgeState,
    SafetyState,
)


class Nav2BridgeNode(Node):
    """Forward Nav2 velocity commands as HumaWare autonomy candidates."""

    def __init__(self) -> None:
        super().__init__("nav2_bridge")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("enabled", True)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.75)
        self.declare_parameter("input_frame_id", "nav2")

        self._active_mode = ModeState.MODE_INACTIVE
        self._safety_state = SafetyState.STATE_UNKNOWN
        self._locomotion_state = LocomotionState.STATE_UNKNOWN
        self._last_input_time: Optional[Time] = None
        self._last_output_time: Optional[Time] = None
        self._latest_command: Optional[TwistStamped] = None
        self._state = NavigationBridgeState.STATE_IDLE
        self._reason = "waiting_for_nav2_command"
        self._output_enabled = False

        self._cmd_pub = self.create_publisher(TwistStamped, "autonomy/cmd_vel", 10)
        self._state_pub = self.create_publisher(
            NavigationBridgeState,
            "navigation/nav2_bridge_state",
            10,
        )

        self.create_subscription(Twist, "nav2/cmd_vel", self._on_twist, 10)
        self.create_subscription(TwistStamped, "nav2/cmd_vel_stamped", self._on_twist_stamped, 10)
        self.create_subscription(ModeState, "mode/state", self._on_mode_state, 10)
        self.create_subscription(SafetyState, "safety/state", self._on_safety_state, 10)
        self.create_subscription(
            LocomotionState,
            "locomotion/state",
            self._on_locomotion_state,
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._tick)

    def _on_mode_state(self, msg: ModeState) -> None:
        self._active_mode = msg.active_mode

    def _on_safety_state(self, msg: SafetyState) -> None:
        self._safety_state = msg.state

    def _on_locomotion_state(self, msg: LocomotionState) -> None:
        self._locomotion_state = msg.state

    def _on_twist(self, msg: Twist) -> None:
        now = self.get_clock().now()
        stamped = TwistStamped()
        stamped.header.stamp = now.to_msg()
        stamped.header.frame_id = str(self.get_parameter("input_frame_id").value)
        stamped.twist = msg
        self._handle_command(stamped, now)

    def _on_twist_stamped(self, msg: TwistStamped) -> None:
        now = self.get_clock().now()
        stamped = TwistStamped()
        stamped.header = msg.header
        if not stamped.header.frame_id:
            stamped.header.frame_id = str(self.get_parameter("input_frame_id").value)
        stamped.header.stamp = now.to_msg()
        stamped.twist = msg.twist
        self._handle_command(stamped, now)

    def _handle_command(self, msg: TwistStamped, now: Time) -> None:
        self._latest_command = msg
        self._last_input_time = now
        self._update_bridge_state(now)
        if self._output_enabled:
            self._cmd_pub.publish(msg)
            self._last_output_time = now

    def _tick(self) -> None:
        now = self.get_clock().now()
        self._update_bridge_state(now)
        self._publish_state(now)

    def _update_bridge_state(self, now: Time) -> None:
        if not bool(self.get_parameter("enabled").value):
            self._state = NavigationBridgeState.STATE_DISABLED
            self._reason = "bridge_disabled"
            self._output_enabled = False
            return

        if self._latest_command is None or self._last_input_time is None:
            self._state = NavigationBridgeState.STATE_IDLE
            self._reason = "waiting_for_nav2_command"
            self._output_enabled = False
            return

        if now - self._last_input_time > Duration(
            seconds=float(self.get_parameter("command_timeout_s").value)
        ):
            self._state = NavigationBridgeState.STATE_STALE
            self._reason = "nav2_command_stale"
            self._output_enabled = False
            return

        if self._active_mode != ModeState.MODE_AUTONOMY:
            self._state = NavigationBridgeState.STATE_BLOCKED
            self._reason = "mode_not_autonomy"
            self._output_enabled = False
            return

        if self._safety_state not in (
            SafetyState.STATE_OK,
            SafetyState.STATE_WARN,
        ):
            self._state = NavigationBridgeState.STATE_BLOCKED
            self._reason = "safety_state_blocks_navigation"
            self._output_enabled = False
            return

        if self._locomotion_state in (
            LocomotionState.STATE_UNKNOWN,
            LocomotionState.STATE_FAULT,
            LocomotionState.STATE_RECOVERING,
        ):
            self._state = NavigationBridgeState.STATE_BLOCKED
            self._reason = "locomotion_not_ready"
            self._output_enabled = False
            return

        self._state = NavigationBridgeState.STATE_ACTIVE
        self._reason = "forwarding_nav2_command"
        self._output_enabled = True

    def _publish_state(self, now: Time) -> None:
        robot_id = str(self.get_parameter("robot_id").value)
        state = NavigationBridgeState()
        state.header.stamp = now.to_msg()
        state.header.frame_id = robot_id
        state.robot_id = robot_id
        state.state = self._state
        state.active_mode = self._active_mode
        state.safety_state = self._safety_state
        state.locomotion_state = self._locomotion_state
        state.bridge_enabled = bool(self.get_parameter("enabled").value)
        state.output_enabled = self._output_enabled
        state.input_topic = "nav2/cmd_vel"
        state.output_topic = "autonomy/cmd_vel"
        if self._last_input_time is not None:
            state.last_input_time = self._last_input_time.to_msg()
            state.command_age = self._duration_msg(now - self._last_input_time)
        if self._last_output_time is not None:
            state.last_output_time = self._last_output_time.to_msg()
        state.reason = self._reason
        self._state_pub.publish(state)

    @staticmethod
    def _duration_msg(duration: Duration) -> DurationMsg:
        msg = DurationMsg()
        total_nanoseconds = duration.nanoseconds
        msg.sec = int(total_nanoseconds // 1_000_000_000)
        msg.nanosec = int(total_nanoseconds % 1_000_000_000)
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
