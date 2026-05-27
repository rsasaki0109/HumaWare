from humaware_capability_registry.capability_registry_node import (
    DEFAULT_CAPABILITIES,
    capability_names,
    capability_state,
)
from humaware_msgs.msg import Capability, LocomotionState, ModeState, SafetyState


def spec(name):
    for capability_spec in DEFAULT_CAPABILITIES:
        if capability_spec.name == name:
            return capability_spec
    raise AssertionError(f"missing capability spec: {name}")


def test_default_capabilities_include_runtime_core_contracts():
    assert capability_names(DEFAULT_CAPABILITIES) == {
        "stand",
        "stop",
        "walk_velocity",
        "turn_in_place",
        "recover_posture",
        "request_teleop",
    }


def test_stop_remains_available_during_fault():
    assert (
        capability_state(
            spec("stop"),
            ModeState.MODE_AUTONOMY,
            SafetyState.STATE_FAULT,
            LocomotionState.STATE_WALKING,
        )
        == Capability.STATE_IDLE
    )


def test_motion_requires_active_motion_mode():
    assert (
        capability_state(
            spec("walk_velocity"),
            ModeState.MODE_INACTIVE,
            SafetyState.STATE_OK,
            LocomotionState.STATE_STANDING,
        )
        == Capability.STATE_UNAVAILABLE
    )


def test_motion_reports_executing_when_robot_is_walking():
    assert (
        capability_state(
            spec("walk_velocity"),
            ModeState.MODE_AUTONOMY,
            SafetyState.STATE_OK,
            LocomotionState.STATE_WALKING,
        )
        == Capability.STATE_EXECUTING
    )
