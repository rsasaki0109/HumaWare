"""Unit tests for the pure-function arbitration helpers."""

from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.time import Time

from humaware_command_arbiter.command_arbiter_node import (
    CandidateCommand,
    clamp_velocity,
    select_candidate,
    should_publish_stop,
)
from humaware_msgs.msg import CommandArbitrationState, ModeState, SafetyState


COMMAND_TIMEOUT = Duration(seconds=0.5)


def _twist(linear_x: float = 0.1, angular_z: float = 0.0) -> TwistStamped:
    msg = TwistStamped()
    msg.header.frame_id = "test"
    msg.twist.linear.x = linear_x
    msg.twist.angular.z = angular_z
    return msg


def _candidate(source: int, required_mode: int, received_at_ns: int, name: str) -> CandidateCommand:
    return CandidateCommand(
        source=source,
        name=name,
        required_mode=required_mode,
        msg=_twist(),
        received_at=Time(nanoseconds=received_at_ns),
    )


def test_select_returns_none_when_safety_unseen():
    candidate, reason = select_candidate(
        active_mode=ModeState.MODE_TELEOP,
        safety_seen=False,
        safety_state=SafetyState.STATE_UNKNOWN,
        candidates={},
        now=Time(nanoseconds=1_000_000_000),
        command_timeout=COMMAND_TIMEOUT,
    )

    assert candidate is None
    assert reason == "waiting_for_safety_state"


def test_select_blocked_by_estop_even_with_fresh_command():
    candidate = _candidate(
        source=CommandArbitrationState.SOURCE_TELEOP,
        required_mode=ModeState.MODE_TELEOP,
        received_at_ns=1_000_000_000,
        name="teleop",
    )

    selected, reason = select_candidate(
        active_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_ESTOP,
        candidates={CommandArbitrationState.SOURCE_TELEOP: candidate},
        now=Time(nanoseconds=1_000_000_000),
        command_timeout=COMMAND_TIMEOUT,
    )

    assert selected is None
    assert reason == "safety_state_blocks_output"


def test_select_blocked_by_mrm():
    candidate = _candidate(
        source=CommandArbitrationState.SOURCE_TELEOP,
        required_mode=ModeState.MODE_TELEOP,
        received_at_ns=1_000_000_000,
        name="teleop",
    )

    selected, reason = select_candidate(
        active_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_MRM,
        candidates={CommandArbitrationState.SOURCE_TELEOP: candidate},
        now=Time(nanoseconds=1_000_000_000),
        command_timeout=COMMAND_TIMEOUT,
    )

    assert selected is None
    assert reason == "safety_state_blocks_output"


def test_select_blocked_when_mode_inactive():
    candidate = _candidate(
        source=CommandArbitrationState.SOURCE_TELEOP,
        required_mode=ModeState.MODE_TELEOP,
        received_at_ns=1_000_000_000,
        name="teleop",
    )

    selected, reason = select_candidate(
        active_mode=ModeState.MODE_INACTIVE,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        candidates={CommandArbitrationState.SOURCE_TELEOP: candidate},
        now=Time(nanoseconds=1_000_000_000),
        command_timeout=COMMAND_TIMEOUT,
    )

    assert selected is None
    assert reason == "active_mode_blocks_output"


def test_select_rejects_command_for_other_mode():
    autonomy_candidate = _candidate(
        source=CommandArbitrationState.SOURCE_AUTONOMY,
        required_mode=ModeState.MODE_AUTONOMY,
        received_at_ns=1_000_000_000,
        name="autonomy",
    )

    selected, reason = select_candidate(
        active_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        candidates={CommandArbitrationState.SOURCE_AUTONOMY: autonomy_candidate},
        now=Time(nanoseconds=1_000_000_000),
        command_timeout=COMMAND_TIMEOUT,
    )

    assert selected is None
    assert reason == "no_fresh_command_for_active_mode"


def test_select_returns_none_when_command_is_stale():
    candidate = _candidate(
        source=CommandArbitrationState.SOURCE_TELEOP,
        required_mode=ModeState.MODE_TELEOP,
        received_at_ns=0,
        name="teleop",
    )

    selected, reason = select_candidate(
        active_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        candidates={CommandArbitrationState.SOURCE_TELEOP: candidate},
        now=Time(nanoseconds=10 * 1_000_000_000),
        command_timeout=COMMAND_TIMEOUT,
    )

    assert selected is None
    assert reason == "no_fresh_command_for_active_mode"


def test_select_prefers_teleop_over_autonomy_when_both_fresh_in_teleop_mode():
    teleop = _candidate(
        source=CommandArbitrationState.SOURCE_TELEOP,
        required_mode=ModeState.MODE_TELEOP,
        received_at_ns=1_000_000_000,
        name="teleop",
    )
    autonomy = _candidate(
        source=CommandArbitrationState.SOURCE_AUTONOMY,
        required_mode=ModeState.MODE_AUTONOMY,
        received_at_ns=1_000_000_000,
        name="autonomy",
    )

    selected, reason = select_candidate(
        active_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_WARN,
        candidates={
            CommandArbitrationState.SOURCE_TELEOP: teleop,
            CommandArbitrationState.SOURCE_AUTONOMY: autonomy,
        },
        now=Time(nanoseconds=1_000_000_000),
        command_timeout=COMMAND_TIMEOUT,
    )

    assert selected is teleop
    assert reason == "approved"


def test_select_picks_autonomy_when_autonomy_mode_active():
    autonomy = _candidate(
        source=CommandArbitrationState.SOURCE_AUTONOMY,
        required_mode=ModeState.MODE_AUTONOMY,
        received_at_ns=1_000_000_000,
        name="autonomy",
    )

    selected, reason = select_candidate(
        active_mode=ModeState.MODE_AUTONOMY,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        candidates={CommandArbitrationState.SOURCE_AUTONOMY: autonomy},
        now=Time(nanoseconds=1_000_000_000),
        command_timeout=COMMAND_TIMEOUT,
    )

    assert selected is autonomy
    assert reason == "approved"


def test_should_publish_stop_respects_parameter_off():
    assert should_publish_stop("safety_state_blocks_output", publish_stop_on_block=False) is False


def test_should_publish_stop_for_blocked_reasons():
    assert should_publish_stop("safety_state_blocks_output", True) is True
    assert should_publish_stop("active_mode_blocks_output", True) is True
    assert should_publish_stop("no_fresh_command_for_active_mode", True) is True


def test_should_not_publish_stop_for_waiting_safety():
    assert should_publish_stop("waiting_for_safety_state", True) is False


def test_clamp_velocity_limits_linear_and_angular():
    raw = _twist(linear_x=10.0, angular_z=-10.0)
    raw.twist.linear.y = 5.0
    raw.twist.linear.z = 5.0
    raw.twist.angular.x = 5.0
    raw.twist.angular.y = 5.0

    approved = clamp_velocity(
        msg=raw,
        max_linear=0.5,
        max_angular=0.5,
        stamp=raw.header.stamp,
        frame_id="approved",
    )

    assert approved.twist.linear.x == 0.5
    assert approved.twist.linear.y == 0.5
    assert approved.twist.linear.z == 0.0
    assert approved.twist.angular.x == 0.0
    assert approved.twist.angular.y == 0.0
    assert approved.twist.angular.z == -0.5
    assert approved.header.frame_id == "approved"
