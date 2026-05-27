"""Unit tests for the bag evaluator pure helpers."""

from dataclasses import dataclass

from humaware_bag_evaluator.evaluator import (
    DEFAULT_REQUIRED_TOPICS,
    EvaluationResult,
    format_summary,
    summarize_topic_freshness,
)


def _ns(seconds: float) -> int:
    return int(seconds * 1_000_000_000)


@dataclass
class FakeMode:
    active_mode: int = 0


@dataclass
class FakeSafety:
    state: int = 0


@dataclass
class FakeArbitration:
    active_source: int = 0
    output_enabled: bool = False


def _stream():
    return [
        ("mode/state", _ns(0.0), FakeMode(active_mode=1)),
        ("mode/state", _ns(0.1), FakeMode(active_mode=2)),
        ("safety/state", _ns(0.0), FakeSafety(state=0)),
        ("safety/state", _ns(0.1), FakeSafety(state=0)),
        ("safety/state", _ns(0.2), FakeSafety(state=1)),
        ("runtime/command_arbitration_state", _ns(0.0), FakeArbitration(active_source=0)),
        (
            "runtime/command_arbitration_state",
            _ns(0.1),
            FakeArbitration(active_source=1, output_enabled=True),
        ),
    ]


def test_summarize_counts_messages_per_topic():
    result = summarize_topic_freshness(_stream(), required_topics=[])

    assert result.summaries["mode/state"].message_count == 2
    assert result.summaries["safety/state"].message_count == 3
    assert result.summaries["runtime/command_arbitration_state"].message_count == 2


def test_summarize_records_first_and_last_timestamps():
    result = summarize_topic_freshness(_stream(), required_topics=[])

    safety = result.summaries["safety/state"]
    assert safety.first_timestamp_ns == _ns(0.0)
    assert safety.last_timestamp_ns == _ns(0.2)


def test_summarize_detects_missing_required_topic():
    result = summarize_topic_freshness(
        _stream(), required_topics=["mode/state", "runtime/health"]
    )

    assert result.missing_required_topics == ["runtime/health"]


def test_summarize_detects_excessive_gap():
    messages = [
        ("mode/state", _ns(0.0), FakeMode(active_mode=1)),
        ("mode/state", _ns(2.0), FakeMode(active_mode=1)),
    ]

    result = summarize_topic_freshness(
        messages,
        required_topics=["mode/state"],
        max_gap_ns=_ns(1.0),
    )

    assert result.stale_topics == ["mode/state"]
    assert result.summaries["mode/state"].max_gap_ns == _ns(2.0)


def test_summarize_tracks_mode_state_transitions():
    result = summarize_topic_freshness(_stream(), required_topics=[])

    transitions = [
        event
        for event in result.transitions
        if event.topic == "mode/state" and event.field_name == "active_mode"
    ]
    assert len(transitions) == 1
    assert transitions[0].previous == 1
    assert transitions[0].current == 2


def test_summarize_tracks_safety_state_transitions():
    result = summarize_topic_freshness(_stream(), required_topics=[])

    transitions = [
        event for event in result.transitions if event.topic == "safety/state"
    ]
    assert len(transitions) == 1
    assert transitions[0].previous == 0
    assert transitions[0].current == 1


def test_summarize_tracks_command_arbitration_source_and_output_enabled():
    result = summarize_topic_freshness(_stream(), required_topics=[])

    fields = {
        (event.field_name, event.previous, event.current)
        for event in result.transitions
        if event.topic == "runtime/command_arbitration_state"
    }
    assert ("active_source", 0, 1) in fields
    assert ("output_enabled", False, True) in fields


def test_summarize_strips_robot_namespace_for_watched_fields():
    messages = [
        ("/mock_001/mode/state", _ns(0.0), FakeMode(active_mode=1)),
        ("/mock_001/mode/state", _ns(0.1), FakeMode(active_mode=4)),
    ]

    result = summarize_topic_freshness(messages, required_topics=[])

    transitions = [event for event in result.transitions]
    assert len(transitions) == 1
    assert transitions[0].field_name == "active_mode"
    assert transitions[0].previous == 1
    assert transitions[0].current == 4


def test_format_summary_includes_required_and_stale_sections():
    result = EvaluationResult(
        summaries={},
        missing_required_topics=["mode/state"],
        stale_topics=[],
    )

    output = format_summary(result)
    assert "Missing required topics" in output
    assert "mode/state" in output


def test_default_required_topics_are_consistent_with_safety_and_mode():
    assert "mode/state" in DEFAULT_REQUIRED_TOPICS
    assert "safety/state" in DEFAULT_REQUIRED_TOPICS
    assert "runtime/command_arbitration_state" in DEFAULT_REQUIRED_TOPICS
    assert "cmd_vel/approved" in DEFAULT_REQUIRED_TOPICS
