"""Mock humanoid state publisher."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from humaware_msgs.msg import (
    BalanceState,
    FootContactState,
    HumanoidState,
    LocomotionState,
    ModeState,
    SafetyState,
)


class MockRobotNode(Node):
    """Publish stable mock robot state for integration tests and launch demos."""

    def __init__(self) -> None:
        super().__init__("mock_robot")

        self.declare_parameter("robot_id", "mock_001")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("battery_percentage", 100.0)

        self._active_mode = ModeState.MODE_INACTIVE
        self._locomotion_state = LocomotionState.STATE_UNKNOWN
        self._safety_state = SafetyState.STATE_UNKNOWN
        self._humanoid_pub = self.create_publisher(HumanoidState, "state", 10)
        self._balance_pub = self.create_publisher(BalanceState, "balance/state", 10)
        self._foot_contact_pub = self.create_publisher(FootContactState, "foot_contact/state", 10)
        self.create_subscription(ModeState, "mode/state", self._on_mode_state, 10)
        self.create_subscription(
            LocomotionState,
            "locomotion/state",
            self._on_locomotion_state,
            10,
        )
        self.create_subscription(SafetyState, "safety/state", self._on_safety_state, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._publish_state)

    def _on_mode_state(self, msg: ModeState) -> None:
        self._active_mode = msg.active_mode

    def _on_locomotion_state(self, msg: LocomotionState) -> None:
        self._locomotion_state = msg.state

    def _on_safety_state(self, msg: SafetyState) -> None:
        self._safety_state = msg.state

    def _publish_state(self) -> None:
        now = self.get_clock().now().to_msg()
        robot_id = str(self.get_parameter("robot_id").value)
        battery_percentage = float(self.get_parameter("battery_percentage").value)

        balance = BalanceState()
        balance.header.stamp = now
        balance.header.frame_id = robot_id
        balance.robot_id = robot_id
        balance.state = BalanceState.STATE_STABLE
        balance.stability_margin = 1.0

        foot_contact = FootContactState()
        foot_contact.header.stamp = now
        foot_contact.header.frame_id = robot_id
        foot_contact.robot_id = robot_id
        foot_contact.left_contact = True
        foot_contact.right_contact = True
        foot_contact.left_normal_force_n = 300.0
        foot_contact.right_normal_force_n = 300.0

        humanoid = HumanoidState()
        humanoid.header.stamp = now
        humanoid.header.frame_id = robot_id
        humanoid.robot_id = robot_id
        humanoid.mode = self._active_mode
        humanoid.locomotion_state = self._locomotion_state
        humanoid.safety_state = self._safety_state
        humanoid.balance_state = balance.state
        humanoid.battery_percentage = battery_percentage
        humanoid.cpu_temperature_c = 40.0
        humanoid.operator_present = False
        humanoid.active_capabilities = ["stand", "stop", "walk_velocity"]

        self._balance_pub.publish(balance)
        self._foot_contact_pub.publish(foot_contact)
        self._humanoid_pub.publish(humanoid)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockRobotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
