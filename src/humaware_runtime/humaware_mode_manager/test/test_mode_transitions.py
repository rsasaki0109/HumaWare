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
