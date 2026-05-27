"""Unit tests for the scripted provider candidate-action gate.

The gate must reject every disqualifying input. These tests duplicate
the stub's gate contract on purpose: the contract is canonical, and any
divergence between providers should be caught by tests rather than by
runtime behavior.
"""

from humaware_msgs.msg import ModeState, SafetyState

from humaware_policy_provider_scripted.policy_provider_scripted_node import (
    should_emit_candidate,
)


def test_gate_open_when_ai_policy_and_safety_ok():
    allow, reason = should_emit_candidate(
        enabled=True,
        active_mode=ModeState.MODE_AI_POLICY,
        safety_state=SafetyState.STATE_OK,
        safety_seen=True,
    )

    assert allow is True
    assert reason == "policy_candidate_emitted"


def test_gate_open_when_safety_warn():
    allow, reason = should_emit_candidate(
        enabled=True,
        active_mode=ModeState.MODE_AI_POLICY,
        safety_state=SafetyState.STATE_WARN,
        safety_seen=True,
    )

    assert allow is True
    assert reason == "policy_candidate_emitted"


def test_gate_closed_when_disabled():
    allow, reason = should_emit_candidate(
        enabled=False,
        active_mode=ModeState.MODE_AI_POLICY,
        safety_state=SafetyState.STATE_OK,
        safety_seen=True,
    )

    assert allow is False
    assert reason == "policy_disabled_by_parameter"


def test_gate_closed_when_safety_unseen():
    allow, reason = should_emit_candidate(
        enabled=True,
        active_mode=ModeState.MODE_AI_POLICY,
        safety_state=SafetyState.STATE_UNKNOWN,
        safety_seen=False,
    )

    assert allow is False
    assert reason == "waiting_for_safety_state"


def test_gate_closed_when_safety_estop():
    allow, reason = should_emit_candidate(
        enabled=True,
        active_mode=ModeState.MODE_AI_POLICY,
        safety_state=SafetyState.STATE_ESTOP,
        safety_seen=True,
    )

    assert allow is False
    assert reason == "safety_state_blocks_policy"


def test_gate_closed_when_safety_mrm():
    allow, reason = should_emit_candidate(
        enabled=True,
        active_mode=ModeState.MODE_AI_POLICY,
        safety_state=SafetyState.STATE_MRM,
        safety_seen=True,
    )

    assert allow is False
    assert reason == "safety_state_blocks_policy"


def test_gate_closed_when_active_mode_is_teleop():
    allow, reason = should_emit_candidate(
        enabled=True,
        active_mode=ModeState.MODE_TELEOP,
        safety_state=SafetyState.STATE_OK,
        safety_seen=True,
    )

    assert allow is False
    assert reason == "active_mode_not_ai_policy"


def test_gate_closed_when_active_mode_is_autonomy():
    allow, reason = should_emit_candidate(
        enabled=True,
        active_mode=ModeState.MODE_AUTONOMY,
        safety_state=SafetyState.STATE_OK,
        safety_seen=True,
    )

    assert allow is False
    assert reason == "active_mode_not_ai_policy"


def test_gate_closed_when_active_mode_is_inactive():
    allow, reason = should_emit_candidate(
        enabled=True,
        active_mode=ModeState.MODE_INACTIVE,
        safety_state=SafetyState.STATE_OK,
        safety_seen=True,
    )

    assert allow is False
    assert reason == "active_mode_not_ai_policy"
