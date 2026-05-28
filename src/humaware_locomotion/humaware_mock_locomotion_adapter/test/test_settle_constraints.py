"""Regression tests for locomotion stop-settle constraint reporting.

The mock locomotion adapter keeps all of its state-machine logic inside the
node, so these tests bring up a real rclpy node (parameters must be declared)
and drive the timer-side methods directly with synthetic clock values. Every
method under test already takes ``now`` as an explicit argument, so no real
clock or executor spinning is required.
"""

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.time import Time  # noqa: E402

from humaware_msgs.msg import LocomotionState, ModeState, SafetyState  # noqa: E402
from humaware_mock_locomotion_adapter.mock_locomotion_adapter_node import (  # noqa: E402
    MockLocomotionAdapterNode,
)


def _ns(seconds: float) -> int:
    return int(seconds * 1_000_000_000)


@pytest.fixture()
def node():
    rclpy.init()
    instance = MockLocomotionAdapterNode()
    try:
        yield instance
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_held_under_safety_keeps_constraint_after_settling(node):
    # A walking robot is forced to stop because safety is no longer OK/WARN.
    node._active_mode = ModeState.MODE_AUTONOMY
    node._safety_state = SafetyState.STATE_ESTOP
    node._state = LocomotionState.STATE_WALKING
    node._linear_velocity_mps = 0.3
    node._angular_velocity_radps = 0.0

    # First tick: the stop begins but has not settled yet.
    node._update_state_for_runtime(Time(nanoseconds=_ns(0.0)))
    assert node._state == LocomotionState.STATE_STOPPING
    assert node._constraints == ["safety_blocks_locomotion"]

    # Later tick (past the default 0.4 s settle window): the robot reaches a
    # full stop, but safety still blocks output, so the holding reason must
    # remain -- the robot is *held* standing, not idle.
    node._update_state_for_runtime(Time(nanoseconds=_ns(0.5)))
    assert node._state == LocomotionState.STATE_STANDING
    assert node._constraints == ["safety_blocks_locomotion"]


def test_commanded_stop_clears_constraint_after_settling(node):
    # Runtime is healthy and a fresh (zero) command stopped the robot; once it
    # settles it is free to move again, so the constraint list must clear.
    node._active_mode = ModeState.MODE_TELEOP
    node._safety_state = SafetyState.STATE_OK
    node._state = LocomotionState.STATE_STOPPING
    node._stop_started_at = Time(nanoseconds=_ns(0.0))
    node._constraints = ["approved_stop_command"]
    node._last_command_time = Time(nanoseconds=_ns(0.4))

    node._update_state_for_runtime(Time(nanoseconds=_ns(0.5)))

    assert node._state == LocomotionState.STATE_STANDING
    assert node._constraints == []
