"""Unit tests for the hardware adapter template gate logic."""

from rclpy.duration import Duration
from rclpy.time import Time

from humaware_hardware_adapter_template.template_adapter_node import (
    GATE_OPEN_REASON,
    should_release_output,
)
from humaware_msgs.msg import ModeState, MRMState, SafetyState


APPROVED_TIMEOUT = Duration(seconds=0.5)
ARBITRATION_TIMEOUT = Duration(seconds=1.0)


def _ns(seconds: float) -> int:
    return int(seconds * 1_000_000_000)


def test_gate_opens_when_all_inputs_fresh_and_safe():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_OK,
        mrm_state=MRMState.STATE_NONE,
        active_mode=ModeState.MODE_TELEOP,
        last_approved_at=Time(nanoseconds=_ns(1.0)),
        last_arbitration_at=Time(nanoseconds=_ns(1.0)),
        arbitration_output_enabled=True,
        now=Time(nanoseconds=_ns(1.2)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is True
    assert reason == GATE_OPEN_REASON


def test_gate_closed_by_estop():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_ESTOP,
        mrm_state=MRMState.STATE_NONE,
        active_mode=ModeState.MODE_TELEOP,
        last_approved_at=Time(nanoseconds=_ns(1.0)),
        last_arbitration_at=Time(nanoseconds=_ns(1.0)),
        arbitration_output_enabled=True,
        now=Time(nanoseconds=_ns(1.2)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is False
    assert reason == "safety_state_blocks_output"


def test_gate_closed_when_safety_unknown():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_UNKNOWN,
        mrm_state=MRMState.STATE_NONE,
        active_mode=ModeState.MODE_TELEOP,
        last_approved_at=Time(nanoseconds=_ns(1.0)),
        last_arbitration_at=Time(nanoseconds=_ns(1.0)),
        arbitration_output_enabled=True,
        now=Time(nanoseconds=_ns(1.2)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is False
    assert reason == "safety_state_not_ready"


def test_gate_closed_when_mrm_active():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_WARN,
        mrm_state=MRMState.STATE_STOP,
        active_mode=ModeState.MODE_TELEOP,
        last_approved_at=Time(nanoseconds=_ns(1.0)),
        last_arbitration_at=Time(nanoseconds=_ns(1.0)),
        arbitration_output_enabled=True,
        now=Time(nanoseconds=_ns(1.2)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is False
    assert reason == "mrm_state_blocks_output"


def test_gate_closed_when_mode_inactive():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_OK,
        mrm_state=MRMState.STATE_NONE,
        active_mode=ModeState.MODE_INACTIVE,
        last_approved_at=Time(nanoseconds=_ns(1.0)),
        last_arbitration_at=Time(nanoseconds=_ns(1.0)),
        arbitration_output_enabled=True,
        now=Time(nanoseconds=_ns(1.2)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is False
    assert reason == "active_mode_blocks_output"


def test_gate_closed_when_arbitration_state_missing():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_OK,
        mrm_state=MRMState.STATE_NONE,
        active_mode=ModeState.MODE_TELEOP,
        last_approved_at=Time(nanoseconds=_ns(1.0)),
        last_arbitration_at=None,
        arbitration_output_enabled=True,
        now=Time(nanoseconds=_ns(1.2)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is False
    assert reason == "waiting_for_arbitration_state"


def test_gate_closed_when_arbitration_state_stale():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_OK,
        mrm_state=MRMState.STATE_NONE,
        active_mode=ModeState.MODE_TELEOP,
        last_approved_at=Time(nanoseconds=_ns(5.0)),
        last_arbitration_at=Time(nanoseconds=_ns(1.0)),
        arbitration_output_enabled=True,
        now=Time(nanoseconds=_ns(5.0)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is False
    assert reason == "arbitration_state_stale"


def test_gate_closed_when_arbiter_disabled_output():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_OK,
        mrm_state=MRMState.STATE_NONE,
        active_mode=ModeState.MODE_TELEOP,
        last_approved_at=Time(nanoseconds=_ns(1.0)),
        last_arbitration_at=Time(nanoseconds=_ns(1.0)),
        arbitration_output_enabled=False,
        now=Time(nanoseconds=_ns(1.2)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is False
    assert reason == "arbiter_disabled_output"


def test_gate_closed_when_no_approved_command_yet():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_OK,
        mrm_state=MRMState.STATE_NONE,
        active_mode=ModeState.MODE_TELEOP,
        last_approved_at=None,
        last_arbitration_at=Time(nanoseconds=_ns(1.0)),
        arbitration_output_enabled=True,
        now=Time(nanoseconds=_ns(1.2)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is False
    assert reason == "waiting_for_approved_command"


def test_gate_closed_when_approved_command_stale():
    allow, reason = should_release_output(
        safety_state=SafetyState.STATE_OK,
        mrm_state=MRMState.STATE_NONE,
        active_mode=ModeState.MODE_TELEOP,
        last_approved_at=Time(nanoseconds=_ns(1.0)),
        last_arbitration_at=Time(nanoseconds=_ns(1.8)),
        arbitration_output_enabled=True,
        now=Time(nanoseconds=_ns(2.0)),
        approved_command_timeout=APPROVED_TIMEOUT,
        arbitration_state_timeout=ARBITRATION_TIMEOUT,
    )

    assert allow is False
    assert reason == "approved_command_stale"
