from humaware_mode_manager.mode_manager_node import evaluate_transition
from humaware_msgs.msg import ModeState, SafetyState


def test_inactive_cannot_enter_ai_policy_directly():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_INACTIVE,
        requested_mode=ModeState.MODE_AI_POLICY,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=False,
    )

    assert not accepted
    assert message == "ai_policy_requires_active_runtime_mode"


def test_teleop_can_enter_ai_policy_when_safety_allows():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_TELEOP,
        requested_mode=ModeState.MODE_AI_POLICY,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=False,
    )

    assert accepted
    assert message == "mode accepted"


def test_autonomy_can_enter_ai_policy_when_safety_warns():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_AUTONOMY,
        requested_mode=ModeState.MODE_AI_POLICY,
        safety_seen=True,
        safety_state=SafetyState.STATE_WARN,
        takeover=False,
    )

    assert accepted
    assert message == "mode accepted"


def test_inactive_can_enter_teleop_when_safety_seen():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_INACTIVE,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=False,
    )

    assert accepted
    assert message == "mode accepted"


def test_ai_policy_remains_blocked_by_mrm():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_TELEOP,
        requested_mode=ModeState.MODE_AI_POLICY,
        safety_seen=True,
        safety_state=SafetyState.STATE_MRM,
        takeover=False,
    )

    assert not accepted
    assert message == "safety_state_blocks_autonomy"


def test_takeover_from_autonomy_succeeds():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_AUTONOMY,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=True,
    )

    assert accepted
    assert message == "operator_takeover_accepted"


def test_takeover_from_ai_policy_succeeds():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_AI_POLICY,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=True,
    )

    assert accepted
    assert message == "operator_takeover_accepted"


def test_takeover_bypasses_safety_state_when_already_active():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_AI_POLICY,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_MRM,
        takeover=True,
    )

    assert accepted
    assert message == "operator_takeover_accepted"


def test_takeover_does_not_require_safety_seen_when_already_active():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_AUTONOMY,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=False,
        safety_state=SafetyState.STATE_UNKNOWN,
        takeover=True,
    )

    assert accepted
    assert message == "operator_takeover_accepted"


def test_takeover_from_teleop_is_a_no_op():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_TELEOP,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=True,
    )

    assert accepted
    assert message == "mode_unchanged"


def test_takeover_from_inactive_rejected():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_INACTIVE,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=True,
    )

    assert not accepted
    assert message == "takeover_requires_autonomy_ai_or_teleop"


def test_takeover_from_maintenance_rejected():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_MAINTENANCE,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=True,
    )

    assert not accepted
    assert message == "takeover_requires_autonomy_ai_or_teleop"


def test_takeover_from_fault_rejected():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_FAULT,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=True,
    )

    assert not accepted
    assert message == "takeover_requires_autonomy_ai_or_teleop"


def test_takeover_blocked_after_shutdown():
    accepted, message = evaluate_transition(
        active_mode=ModeState.MODE_SHUTDOWN,
        requested_mode=ModeState.MODE_TELEOP,
        safety_seen=True,
        safety_state=SafetyState.STATE_OK,
        takeover=True,
    )

    assert not accepted
    assert message == "shutdown_is_terminal"
