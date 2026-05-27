"""Keyboard teleoperation node for HumaWare."""

import select
import sys
import termios
import threading
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from humaware_keyboard_teleop.keymap import (
    HELP_KEYS,
    TeleopVelocity,
    apply_key,
    is_motion_key,
)


HELP_TEXT = """
HumaWare keyboard teleop

w/s: forward/back
a/d: turn left/right
x or space: stop
h or ?: help
Ctrl-C: exit
"""


class KeyboardTeleopNode(Node):
    """Publish keyboard operator commands to teleop/cmd_vel."""

    def __init__(self) -> None:
        super().__init__("keyboard_teleop")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("linear_step_mps", 0.05)
        self.declare_parameter("angular_step_radps", 0.1)
        self.declare_parameter("max_linear_velocity_mps", 0.5)
        self.declare_parameter("max_angular_velocity_radps", 0.5)
        self.declare_parameter("key_timeout_s", 0.5)
        self.declare_parameter("print_help_on_start", True)

        self._velocity = TeleopVelocity()
        self._last_key_at: Optional[Time] = None
        self._stop_sent = True
        self._running = True
        self._terminal_settings = None

        self._cmd_pub = self.create_publisher(TwistStamped, "teleop/cmd_vel", 10)
        self._heartbeat_pub = self.create_publisher(Header, "teleop/heartbeat", 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._publish_tick)

        if bool(self.get_parameter("print_help_on_start").value):
            self.get_logger().info(HELP_TEXT)

        if sys.stdin.isatty():
            self._start_keyboard_thread()
        else:
            self.get_logger().warning(
                "stdin is not a TTY; keyboard teleop is idle. "
                "Run this node in an interactive terminal to send teleop commands."
            )

    def destroy_node(self) -> bool:
        self._running = False
        self._restore_terminal()
        return super().destroy_node()

    def _start_keyboard_thread(self) -> None:
        self._terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        thread = threading.Thread(target=self._read_keys, daemon=True)
        thread.start()

    def _read_keys(self) -> None:
        while self._running and rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue
            key = sys.stdin.read(1)
            if key in HELP_KEYS:
                self.get_logger().info(HELP_TEXT)
                continue
            if not is_motion_key(key):
                continue
            self._apply_key(key)

    def _apply_key(self, key: str) -> None:
        max_angular = float(self.get_parameter("max_angular_velocity_radps").value)
        self._velocity = apply_key(
            key=key,
            velocity=self._velocity,
            linear_step_mps=float(self.get_parameter("linear_step_mps").value),
            angular_step_radps=float(self.get_parameter("angular_step_radps").value),
            max_linear_velocity_mps=float(self.get_parameter("max_linear_velocity_mps").value),
            max_angular_velocity_radps=max_angular,
        )
        self._last_key_at = self.get_clock().now()
        if self._velocity == TeleopVelocity():
            self._publish_command(self._last_key_at)
            self._stop_sent = True
        else:
            self._stop_sent = False

    def _publish_tick(self) -> None:
        now = self.get_clock().now()
        self._publish_heartbeat(now)

        if self._is_timed_out(now):
            self._velocity = TeleopVelocity()

        if self._velocity == TeleopVelocity():
            if not self._stop_sent:
                self._publish_command(now)
                self._stop_sent = True
            return

        self._publish_command(now)
        self._stop_sent = False

    def _is_timed_out(self, now: Time) -> bool:
        if self._last_key_at is None:
            return False
        timeout_s = float(self.get_parameter("key_timeout_s").value)
        return now - self._last_key_at > Duration(seconds=timeout_s)

    def _publish_command(self, now: Time) -> None:
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = str(self.get_parameter("robot_id").value)
        msg.twist.linear.x = self._velocity.linear_x_mps
        msg.twist.angular.z = self._velocity.angular_z_radps
        self._cmd_pub.publish(msg)

    def _publish_heartbeat(self, now: Time) -> None:
        msg = Header()
        msg.stamp = now.to_msg()
        msg.frame_id = str(self.get_parameter("robot_id").value)
        self._heartbeat_pub.publish(msg)

    def _restore_terminal(self) -> None:
        if self._terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
            self._terminal_settings = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
