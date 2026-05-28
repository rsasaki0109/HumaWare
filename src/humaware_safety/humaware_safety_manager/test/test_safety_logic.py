"""Unit tests for pure-function safety logic helpers."""

from rclpy.duration import Duration
from rclpy.time import Time

from humaware_msgs.msg import ModeState
from humaware_safety_manager.safety_manager_node import (
    HeartbeatPolicy,
    check_heartbeat,
    evaluate_watchdogs,
    select_mrm_reason,
)


TELEOP_POLICY = HeartbeatPolicy(
    name="teleop_heartbeat",
    timeout=Duration(seconds=1.0),
    trigger_mrm=False,
)
TELEOP_POLICY_MRM = HeartbeatPolicy(
    name="teleop_heartbeat",
    timeout=Duration(seconds=1.0),
    trigger_mrm=True,
)
HARDWARE_POLICY = HeartbeatPolicy(
    name="hardware_heartbeat",
    timeout=Duration(seconds=1.0),
    trigger_mrm=True,
)
APPROVED_TIMEOUT = Duration(seconds=1.0)


def _ns(seconds: float) -> int:
    return int(seconds * 1_000_000_000)


def test_check_heartbeat_missing_marks_reason():
    finding = check_heartbeat(
        now=Time(nanoseconds=_ns(5.0)),
        last_seen=None,
        policy=TELEOP_POLICY,
    )

    assert finding == ("teleop_heartbeat_missing", False)


def test_check_heartbeat_fresh_returns_none():
    finding = check_heartbeat(
        now=Time(nanoseconds=_ns(1.0)),
        last_seen=Time(nanoseconds=_ns(0.9)),
        policy=TELEOP_POLICY,
    )

    assert finding is None


def test_check_heartbeat_timeout_marks_reason():
    finding = check_heartbeat(
        now=Time(nanoseconds=_ns(5.0)),
        last_seen=Time(nanoseconds=_ns(1.0)),
        policy=HARDWARE_POLICY,
    )

    assert finding == ("hardware_heartbeat_timeout", True)


def test_watchdogs_quiet_when_inactive_mode_and_nothing_required():
    result = evaluate_watchdogs(
        now=Time(nanoseconds=_ns(5.0)),
        active_mode=ModeState.MODE_INACTIVE,
        teleop_last_seen=None,
        hardware_last_seen=None,
        approved_last_seen=None,
        require_teleop_heartbeat=True,
        teleop_policy=TELEOP_POLICY,
        require_hardware_heartbeat=False,
        hardware_policy=HARDWARE_POLICY,
        monitor_approved_commands=True,
        approved_command_timeout=APPROVED_TIMEOUT,
        approved_command_timeout_triggers_mrm=False,
    )

    assert result.warnings == []
    assert result.mrm_reasons == []


def test_watchdogs_warn_on_missing_teleop_heartbeat_in_teleop_mode():
    result = evaluate_watchdogs(
        now=Time(nanoseconds=_ns(5.0)),
        active_mode=ModeState.MODE_TELEOP,
        teleop_last_seen=None,
        hardware_last_seen=None,
        approved_last_seen=None,
        require_teleop_heartbeat=True,
        teleop_policy=TELEOP_POLICY,
        require_hardware_heartbeat=False,
        hardware_policy=HARDWARE_POLICY,
        monitor_approved_commands=False,
        approved_command_timeout=APPROVED_TIMEOUT,
        approved_command_timeout_triggers_mrm=False,
    )

    assert result.warnings == ["teleop_heartbeat_missing"]
    assert result.mrm_reasons == []


def test_watchdogs_trigger_mrm_on_teleop_heartbeat_when_policy_says_so():
    result = evaluate_watchdogs(
        now=Time(nanoseconds=_ns(5.0)),
        active_mode=ModeState.MODE_TELEOP,
        teleop_last_seen=Time(nanoseconds=_ns(0.5)),
        hardware_last_seen=None,
        approved_last_seen=None,
        require_teleop_heartbeat=True,
        teleop_policy=TELEOP_POLICY_MRM,
        require_hardware_heartbeat=False,
        hardware_policy=HARDWARE_POLICY,
        monitor_approved_commands=False,
        approved_command_timeout=APPROVED_TIMEOUT,
        approved_command_timeout_triggers_mrm=False,
    )

    assert result.warnings == []
    assert result.mrm_reasons == ["teleop_heartbeat_timeout"]


def test_watchdogs_check_hardware_heartbeat_in_autonomy():
    result = evaluate_watchdogs(
        now=Time(nanoseconds=_ns(5.0)),
        active_mode=ModeState.MODE_AUTONOMY,
        teleop_last_seen=None,
        hardware_last_seen=None,
        approved_last_seen=None,
        require_teleop_heartbeat=True,
        teleop_policy=TELEOP_POLICY,
        require_hardware_heartbeat=True,
        hardware_policy=HARDWARE_POLICY,
        monitor_approved_commands=False,
        approved_command_timeout=APPROVED_TIMEOUT,
        approved_command_timeout_triggers_mrm=True,
    )

    assert result.warnings == []
    assert result.mrm_reasons == ["hardware_heartbeat_missing"]


def test_watchdogs_skip_teleop_heartbeat_when_not_in_teleop_mode():
    result = evaluate_watchdogs(
        now=Time(nanoseconds=_ns(5.0)),
        active_mode=ModeState.MODE_AUTONOMY,
        teleop_last_seen=None,
        hardware_last_seen=None,
        approved_last_seen=None,
        require_teleop_heartbeat=True,
        teleop_policy=TELEOP_POLICY,
        require_hardware_heartbeat=False,
        hardware_policy=HARDWARE_POLICY,
        monitor_approved_commands=False,
        approved_command_timeout=APPROVED_TIMEOUT,
        approved_command_timeout_triggers_mrm=False,
    )

    assert result.warnings == []
    assert result.mrm_reasons == []


def test_watchdogs_warn_on_stale_approved_command_in_active_mode():
    result = evaluate_watchdogs(
        now=Time(nanoseconds=_ns(5.0)),
        active_mode=ModeState.MODE_TELEOP,
        teleop_last_seen=Time(nanoseconds=_ns(4.5)),
        hardware_last_seen=None,
        approved_last_seen=Time(nanoseconds=_ns(2.0)),
        require_teleop_heartbeat=True,
        teleop_policy=TELEOP_POLICY,
        require_hardware_heartbeat=False,
        hardware_policy=HARDWARE_POLICY,
        monitor_approved_commands=True,
        approved_command_timeout=APPROVED_TIMEOUT,
        approved_command_timeout_triggers_mrm=False,
    )

    assert result.warnings == ["approved_command_timeout"]
    assert result.mrm_reasons == []


def test_watchdogs_mrm_when_approved_command_policy_says_so():
    result = evaluate_watchdogs(
        now=Time(nanoseconds=_ns(5.0)),
        active_mode=ModeState.MODE_AI_POLICY,
        teleop_last_seen=None,
        hardware_last_seen=None,
        approved_last_seen=Time(nanoseconds=_ns(2.0)),
        require_teleop_heartbeat=False,
        teleop_policy=TELEOP_POLICY,
        require_hardware_heartbeat=False,
        hardware_policy=HARDWARE_POLICY,
        monitor_approved_commands=True,
        approved_command_timeout=APPROVED_TIMEOUT,
        approved_command_timeout_triggers_mrm=True,
    )

    assert result.warnings == []
    assert result.mrm_reasons == ["approved_command_timeout"]


def test_watchdogs_skip_approved_command_when_no_command_ever_seen():
    # Intentional asymmetry with the heartbeat watchdogs: a never-seen approved
    # command (approved_last_seen is None) must NOT fire, because the arbiter
    # only produces approved commands once it is emitting in an active mode --
    # a None here is the normal boot / mode-entry transient, not a fault.
    # Firing on None would raise a spurious warning, or (under the MRM policy)
    # a spurious MRM that blocks startup. A genuinely dead arbiter is caught
    # downstream by the locomotion and hardware adapter gates. This watchdog is
    # a staleness detector, not a presence detector; see evaluate_watchdogs.
    result = evaluate_watchdogs(
        now=Time(nanoseconds=_ns(5.0)),
        active_mode=ModeState.MODE_TELEOP,
        teleop_last_seen=Time(nanoseconds=_ns(4.5)),
        hardware_last_seen=None,
        approved_last_seen=None,
        require_teleop_heartbeat=True,
        teleop_policy=TELEOP_POLICY,
        require_hardware_heartbeat=False,
        hardware_policy=HARDWARE_POLICY,
        monitor_approved_commands=True,
        approved_command_timeout=APPROVED_TIMEOUT,
        approved_command_timeout_triggers_mrm=False,
    )

    assert result.warnings == []
    assert result.mrm_reasons == []


def test_select_mrm_reason_prefers_parameter_reason():
    reason = select_mrm_reason(
        parameter_mrm_active=True,
        parameter_mrm_reason="config_demands_stop",
        service_mrm_active=True,
        service_mrm_reason="operator",
        watchdog_mrm_reasons=["hardware_heartbeat_timeout"],
    )

    assert reason == "config_demands_stop"


def test_select_mrm_reason_falls_back_to_service_reason():
    reason = select_mrm_reason(
        parameter_mrm_active=False,
        parameter_mrm_reason="",
        service_mrm_active=True,
        service_mrm_reason="operator_pressed_button",
        watchdog_mrm_reasons=[],
    )

    assert reason == "operator_pressed_button"


def test_select_mrm_reason_falls_back_to_watchdog():
    reason = select_mrm_reason(
        parameter_mrm_active=False,
        parameter_mrm_reason="",
        service_mrm_active=False,
        service_mrm_reason="",
        watchdog_mrm_reasons=["hardware_heartbeat_timeout"],
    )

    assert reason == "hardware_heartbeat_timeout"


def test_select_mrm_reason_returns_empty_when_no_source_active():
    reason = select_mrm_reason(
        parameter_mrm_active=False,
        parameter_mrm_reason="",
        service_mrm_active=False,
        service_mrm_reason="",
        watchdog_mrm_reasons=[],
    )

    assert reason == ""


def test_select_mrm_reason_default_label_when_only_parameter_flag_set():
    reason = select_mrm_reason(
        parameter_mrm_active=True,
        parameter_mrm_reason="",
        service_mrm_active=False,
        service_mrm_reason="",
        watchdog_mrm_reasons=[],
    )

    assert reason == "parameter_mrm_active"
