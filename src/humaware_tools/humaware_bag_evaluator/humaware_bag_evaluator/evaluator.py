"""Pure-function helpers for HumaWare runtime bag evaluation.

The reading side of the tool wraps ``rosbag2_py``. This module keeps the
analysis itself dependency-free so the rules can be exercised in unit
tests without a real bag on disk.
"""

from dataclasses import dataclass, field
from typing import Iterable, Optional


DEFAULT_REQUIRED_TOPICS = (
    "mode/state",
    "mode/transition_state",
    "safety/state",
    "safety/mrm_state",
    "locomotion/state",
    "runtime/command_arbitration_state",
    "runtime/health",
    "cmd_vel/approved",
)


@dataclass
class TopicSummary:
    """Aggregated stats for a single topic over the bag."""

    topic: str
    message_count: int = 0
    first_timestamp_ns: Optional[int] = None
    last_timestamp_ns: Optional[int] = None
    max_gap_ns: int = 0


@dataclass
class TransitionEvent:
    """A change in a categorical field on a runtime topic."""

    topic: str
    field_name: str
    previous: object
    current: object
    timestamp_ns: int


@dataclass
class EvaluationResult:
    """Combined summary returned by :func:`evaluate_messages`."""

    summaries: dict[str, TopicSummary] = field(default_factory=dict)
    transitions: list[TransitionEvent] = field(default_factory=list)
    missing_required_topics: list[str] = field(default_factory=list)
    stale_topics: list[str] = field(default_factory=list)


def summarize_topic_freshness(
    messages: Iterable[tuple[str, int, object]],
    required_topics: Iterable[str] = DEFAULT_REQUIRED_TOPICS,
    max_gap_ns: int = 1_500_000_000,
) -> EvaluationResult:
    """Walk an iterable of ``(topic, timestamp_ns, message)`` tuples.

    Returns per-topic message counts, first and last timestamps, the
    largest publication gap observed, the set of required topics that
    were never observed, and the set of topics whose largest gap
    exceeded ``max_gap_ns``.
    """
    summaries: dict[str, TopicSummary] = {}
    last_timestamp: dict[str, int] = {}
    transitions: list[TransitionEvent] = []
    previous_values: dict[tuple[str, str], object] = {}

    for topic, timestamp_ns, message in messages:
        summary = summaries.setdefault(topic, TopicSummary(topic=topic))
        summary.message_count += 1
        if summary.first_timestamp_ns is None:
            summary.first_timestamp_ns = timestamp_ns
        if topic in last_timestamp:
            gap = timestamp_ns - last_timestamp[topic]
            if gap > summary.max_gap_ns:
                summary.max_gap_ns = gap
        last_timestamp[topic] = timestamp_ns
        summary.last_timestamp_ns = timestamp_ns

        for field_name in _watched_fields(topic):
            current = getattr(message, field_name, None)
            if current is None:
                continue
            key = (topic, field_name)
            if key in previous_values and previous_values[key] != current:
                transitions.append(
                    TransitionEvent(
                        topic=topic,
                        field_name=field_name,
                        previous=previous_values[key],
                        current=current,
                        timestamp_ns=timestamp_ns,
                    )
                )
            previous_values[key] = current

    required = list(required_topics)
    observed_topics = list(summaries)
    missing_required = [
        topic
        for topic in required
        if not _required_topic_present(topic, observed_topics)
    ]
    stale = sorted(
        topic for topic, summary in summaries.items() if summary.max_gap_ns > max_gap_ns
    )

    return EvaluationResult(
        summaries=summaries,
        transitions=transitions,
        missing_required_topics=missing_required,
        stale_topics=stale,
    )


WATCHED_FIELDS_BY_TOPIC: dict[str, tuple[str, ...]] = {
    "mode/state": ("active_mode",),
    "mode/transition_state": ("outcome",),
    "safety/state": ("state",),
    "safety/mrm_state": ("state",),
    "runtime/command_arbitration_state": ("active_source", "output_enabled"),
    "runtime/health": ("state",),
    "locomotion/state": ("state",),
}


def _watched_fields(topic: str) -> tuple[str, ...]:
    """Return the set of categorical fields tracked for state transitions."""
    return WATCHED_FIELDS_BY_TOPIC.get(_normalize_topic(topic), ())


def _required_topic_present(required_topic: str, observed_topics: Iterable[str]) -> bool:
    """Return True when ``required_topic`` was observed in the bag.

    Bags record fully-qualified topic names, which under a runtime
    namespace look like ``/<robot_id>/safety/state`` while the required
    topics are namespace-relative (``safety/state``). A required topic is
    considered present when an observed topic equals it or ends with
    ``/<required_topic>`` (the leading slash guards against matching a
    partial trailing segment such as ``automode/state``).
    """
    suffix = "/" + required_topic
    for observed in observed_topics:
        stripped = observed.lstrip("/")
        if stripped == required_topic or stripped.endswith(suffix):
            return True
    return False


def _normalize_topic(topic: str) -> str:
    """Strip an optional leading ``/<robot_id>/`` prefix from a topic name."""
    stripped = topic.lstrip("/")
    parts = stripped.split("/", 1)
    if len(parts) == 2 and parts[1] in WATCHED_FIELDS_BY_TOPIC:
        return parts[1]
    return stripped


def format_summary(result: EvaluationResult) -> str:
    """Return a human-readable summary of an evaluation result."""
    lines: list[str] = ["Runtime bag evaluation summary", "="]
    if result.missing_required_topics:
        lines.append("Missing required topics:")
        for topic in result.missing_required_topics:
            lines.append(f"  - {topic}")
    else:
        lines.append("All required topics present.")

    if result.stale_topics:
        lines.append("Topics with excessive publication gaps:")
        for topic in result.stale_topics:
            summary = result.summaries[topic]
            gap_ms = summary.max_gap_ns / 1e6
            lines.append(f"  - {topic} (max gap {gap_ms:.0f} ms)")
    else:
        lines.append("No topics exceeded the configured gap threshold.")

    lines.append("Per-topic counts:")
    for topic in sorted(result.summaries):
        summary = result.summaries[topic]
        lines.append(
            f"  - {topic}: {summary.message_count} messages,"
            f" max gap {summary.max_gap_ns / 1e6:.0f} ms"
        )

    if result.transitions:
        lines.append("State transitions:")
        for event in result.transitions:
            lines.append(
                f"  - {event.topic}.{event.field_name}:"
                f" {event.previous} -> {event.current}"
                f" @ {event.timestamp_ns}"
            )
    else:
        lines.append("No tracked state transitions observed.")

    return "\n".join(lines)
