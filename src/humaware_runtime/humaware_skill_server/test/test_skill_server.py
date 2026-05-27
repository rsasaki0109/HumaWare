from humaware_msgs.msg import Capability, ModeState, SafetyState, SkillExecutionState

from humaware_skill_server.skill_server_node import evaluate_request, sanitize_velocity_command


def capability(name: str, state: int = Capability.STATE_IDLE) -> Capability:
    msg = Capability()
    msg.name = name
    msg.state = state
    return msg


def test_stop_is_accepted_even_when_safety_is_faulted():
    decision = evaluate_request(
        "stop",
        capability("stop"),
        ModeState.MODE_AUTONOMY,
        SafetyState.STATE_FAULT,
    )

    assert decision.accepted
    assert decision.status == SkillExecutionState.STATUS_ACCEPTED


def test_walk_velocity_requires_ai_policy_mode():
    decision = evaluate_request(
        "walk_velocity",
        capability("walk_velocity"),
        ModeState.MODE_AUTONOMY,
        SafetyState.STATE_OK,
    )

    assert not decision.accepted
    assert decision.message == "skill_requires_ai_policy_mode"


def test_unavailable_capability_is_rejected():
    decision = evaluate_request(
        "walk_velocity",
        capability("walk_velocity", Capability.STATE_UNAVAILABLE),
        ModeState.MODE_AI_POLICY,
        SafetyState.STATE_OK,
    )

    assert not decision.accepted
    assert decision.message == "capability_not_available"


def test_turn_only_command_removes_translation():
    command = sanitize_velocity_command(
        request=velocity_request(linear_x=0.4, angular_z=0.2),
        robot_id="mock_001",
        now=FakeTime(),
        turn_only=True,
    )

    assert command.twist.linear.x == 0.0
    assert command.twist.angular.z == 0.2


def velocity_request(linear_x: float, angular_z: float):
    from geometry_msgs.msg import TwistStamped

    msg = TwistStamped()
    msg.twist.linear.x = linear_x
    msg.twist.angular.z = angular_z
    return msg


class FakeTime:
    def to_msg(self):
        from builtin_interfaces.msg import Time

        return Time()
