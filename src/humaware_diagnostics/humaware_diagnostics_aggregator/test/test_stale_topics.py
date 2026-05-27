"""Unit tests for the diagnostics aggregator stale-topic detection."""

from rclpy.duration import Duration
from rclpy.time import Time

from humaware_msgs.msg import HealthState, LocomotionState, SafetyState
from humaware_diagnostics_aggregator.diagnostics_aggregator_node import (
    TopicSample,
    compute_stale_topics,
    evaluate_health,
)


TIMEOUT = Duration(seconds=1.5)


def _ns(seconds: float) -> int:
    return int(seconds * 1_000_000_000)


def _seen(seconds: float) -> TopicSample:
    return TopicSample(message=object(), received_at=Time(nanoseconds=_ns(seconds)))


def test_missing_topic_is_stale():
    stale = compute_stale_topics(
        samples={"mode/state": TopicSample()},
        required_topics=["mode/state"],
        now=Time(nanoseconds=_ns(5.0)),
        timeout=TIMEOUT,
    )

    assert stale == ["mode/state"]


def test_recently_seen_topic_is_not_stale():
    stale = compute_stale_topics(
        samples={"mode/state": _seen(4.5)},
        required_topics=["mode/state"],
        now=Time(nanoseconds=_ns(5.0)),
        timeout=TIMEOUT,
    )

    assert stale == []


def test_topic_older_than_timeout_is_stale():
    stale = compute_stale_topics(
        samples={"mode/state": _seen(1.0)},
        required_topics=["mode/state"],
        now=Time(nanoseconds=_ns(5.0)),
        timeout=TIMEOUT,
    )

    assert stale == ["mode/state"]


def test_topic_at_exactly_timeout_is_not_stale():
    stale = compute_stale_topics(
        samples={"mode/state": _seen(3.5)},
        required_topics=["mode/state"],
        now=Time(nanoseconds=_ns(5.0)),
        timeout=TIMEOUT,
    )

    assert stale == []


def test_only_required_topics_are_checked():
    stale = compute_stale_topics(
        samples={
            "mode/state": _seen(4.5),
            "safety/state": _seen(0.0),
            "teleop/heartbeat": _seen(0.0),
        },
        required_topics=["mode/state", "safety/state"],
        now=Time(nanoseconds=_ns(5.0)),
        timeout=TIMEOUT,
    )

    assert stale == ["safety/state"]


def test_missing_sample_entry_is_stale():
    stale = compute_stale_topics(
        samples={},
        required_topics=["mode/state"],
        now=Time(nanoseconds=_ns(5.0)),
        timeout=TIMEOUT,
    )

    assert stale == ["mode/state"]


def test_evaluate_health_returns_stale_when_topics_stale():
    state, summary = evaluate_health(
        safety_state=SafetyState.STATE_OK,
        locomotion_state=LocomotionState.STATE_STANDING,
        active_faults=[],
        active_warnings=[],
        stale_topics=["mode/state"],
    )

    assert state == HealthState.HEALTH_STALE
    assert "stale" in summary


def test_evaluate_health_returns_error_on_estop():
    state, summary = evaluate_health(
        safety_state=SafetyState.STATE_ESTOP,
        locomotion_state=LocomotionState.STATE_STANDING,
        active_faults=[],
        active_warnings=[],
        stale_topics=[],
    )

    assert state == HealthState.HEALTH_ERROR
    assert "safety" in summary


def test_evaluate_health_returns_error_on_locomotion_fault():
    state, summary = evaluate_health(
        safety_state=SafetyState.STATE_OK,
        locomotion_state=LocomotionState.STATE_FAULT,
        active_faults=[],
        active_warnings=[],
        stale_topics=[],
    )

    assert state == HealthState.HEALTH_ERROR
    assert "locomotion" in summary


def test_evaluate_health_returns_warn_when_warning_state():
    state, summary = evaluate_health(
        safety_state=SafetyState.STATE_WARN,
        locomotion_state=LocomotionState.STATE_STANDING,
        active_faults=[],
        active_warnings=["teleop_heartbeat_missing"],
        stale_topics=[],
    )

    assert state == HealthState.HEALTH_WARN
    assert "warning" in summary


def test_evaluate_health_returns_ok_when_runtime_healthy():
    state, summary = evaluate_health(
        safety_state=SafetyState.STATE_OK,
        locomotion_state=LocomotionState.STATE_STANDING,
        active_faults=[],
        active_warnings=[],
        stale_topics=[],
    )

    assert state == HealthState.HEALTH_OK
    assert summary == "runtime healthy"
