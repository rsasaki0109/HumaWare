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


def test_summarize_flags_topic_that_dies_before_bag_end():
    # safety/state stops at t=0.5s while cmd_vel/approved runs to t=3.0s.
    # The trailing silence (2.5s) must be flagged even though safety/state's
    # own inter-message gap is small.
    messages = []
    t = 0.0
    while t <= 0.5 + 1e-9:
        messages.append(("safety/state", _ns(t), FakeSafety(state=0)))
        t += 0.1
    t = 0.0
    while t <= 3.0 + 1e-9:
        messages.append(("cmd_vel/approved", _ns(t), FakeSafety(state=0)))
        t += 0.1

    result = summarize_topic_freshness(
        messages,
        required_topics=["safety/state", "cmd_vel/approved"],
        max_gap_ns=_ns(1.5),
    )

    assert "safety/state" in result.stale_topics
    assert "cmd_vel/approved" not in result.stale_topics


def test_summarize_flags_topic_that_starts_late():
    # safety/state only begins at t=2.0s while cmd_vel/approved starts at 0.0;
    # the leading silence must be flagged.
    messages = [
        ("cmd_vel/approved", _ns(0.0), FakeSafety()),
        ("cmd_vel/approved", _ns(2.0), FakeSafety()),
        ("cmd_vel/approved", _ns(2.1), FakeSafety()),
        ("safety/state", _ns(2.0), FakeSafety()),
        ("safety/state", _ns(2.1), FakeSafety()),
    ]

    result = summarize_topic_freshness(
        messages,
        required_topics=["safety/state", "cmd_vel/approved"],
        max_gap_ns=_ns(1.5),
    )

    assert "safety/state" in result.stale_topics


def test_summarize_does_not_flag_window_edge_topics_for_coverage():
    # When every topic spans the full window with tight publication, none
    # should be flagged by the leading/trailing coverage logic.
    messages = []
    for topic in ("safety/state", "cmd_vel/approved"):
        t = 0.0
        while t <= 2.0 + 1e-9:
            messages.append((topic, _ns(t), FakeSafety()))
            t += 0.1

    result = summarize_topic_freshness(
        messages,
        required_topics=["safety/state", "cmd_vel/approved"],
        max_gap_ns=_ns(1.5),
    )

    assert result.stale_topics == []


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


def test_summarize_finds_required_topics_under_robot_namespace():
    # Bags store fully-qualified, namespaced topic names. A healthy bag
    # whose required topics are all present under /<robot_id>/ must not be
    # reported as missing them.
    messages = []
    for topic in DEFAULT_REQUIRED_TOPICS:
        messages.append((f"/mock_001/{topic}", _ns(0.0), FakeSafety()))
        messages.append((f"/mock_001/{topic}", _ns(0.1), FakeSafety()))

    result = summarize_topic_freshness(messages)

    assert result.missing_required_topics == []


def test_summarize_still_reports_missing_topic_under_namespace():
    # Namespace tolerance must not mask a genuinely absent required topic.
    messages = [
        ("/mock_001/mode/state", _ns(0.0), FakeMode(active_mode=1)),
        ("/mock_001/safety/state", _ns(0.0), FakeSafety(state=0)),
    ]

    result = summarize_topic_freshness(
        messages, required_topics=["mode/state", "runtime/health"]
    )

    assert result.missing_required_topics == ["runtime/health"]


def test_summarize_namespace_match_requires_full_trailing_segment():
    # A partial trailing segment (automode/state) must not satisfy
    # the required topic mode/state.
    messages = [("/mock_001/automode/state", _ns(0.0), FakeMode(active_mode=1))]

    result = summarize_topic_freshness(messages, required_topics=["mode/state"])

    assert result.missing_required_topics == ["mode/state"]


def test_format_summary_includes_required_and_stale_sections():
    result = EvaluationResult(
        summaries={},
        missing_required_topics=["mode/state"],
        stale_topics=[],
    )

    output = format_summary(result)
    assert "Missing required topics" in output
    assert "mode/state" in output


def test_format_summary_underlines_title_with_full_rule():
    output = format_summary(EvaluationResult(summaries={}))
    lines = output.splitlines()
    title = "Runtime bag evaluation summary"
    assert lines[0] == title
    # The divider must underline the whole title, not be a lone character.
    assert lines[1] == "=" * len(title)


def test_format_summary_labels_stale_entry_with_coverage_gap():
    from humaware_bag_evaluator.evaluator import TopicSummary

    summary = TopicSummary(
        topic="/mock_001/safety/state",
        message_count=5,
        max_gap_ns=_ns(0.1),
        coverage_gap_ns=_ns(5.0),
    )
    result = EvaluationResult(
        summaries={"/mock_001/safety/state": summary},
        stale_topics=["/mock_001/safety/state"],
    )

    output = format_summary(result)
    # Stale section reports the coverage gap (5000 ms), distinct from the
    # per-topic inter-message "max gap" (100 ms).
    assert "coverage gap 5000 ms" in output
    assert "max gap 100 ms" in output


def test_default_required_topics_are_consistent_with_safety_and_mode():
    assert "mode/state" in DEFAULT_REQUIRED_TOPICS
    assert "safety/state" in DEFAULT_REQUIRED_TOPICS
    assert "runtime/command_arbitration_state" in DEFAULT_REQUIRED_TOPICS
    assert "cmd_vel/approved" in DEFAULT_REQUIRED_TOPICS
