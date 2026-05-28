"""Integration tests that exercise the evaluator against a real rosbag2.

The pure-function tests in ``test_evaluator.py`` feed synthetic tuples
into ``summarize_topic_freshness``. These tests close the loop on the
``cli.evaluate_bag`` reading path by writing an actual rosbag2 with
fully-qualified, namespaced runtime topics and message types, then
evaluating it. They are the end-to-end proof of two fixes that synthetic
tuples cannot fully validate:

* required topics recorded under ``/<robot_id>/`` are recognised as
  present (not falsely reported missing),
* a required stream that dies partway through the recording is flagged
  stale even when its own inter-message cadence was tight.

The tests skip cleanly where ``rosbag2_py`` or the message packages are
unavailable (for example a minimal CI image), so the pure-function suite
still runs everywhere.
"""

import tempfile

import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")
serialization = pytest.importorskip("rclpy.serialization")
humaware_msgs = pytest.importorskip("humaware_msgs.msg")
geometry_msgs = pytest.importorskip("geometry_msgs.msg")

from rclpy.serialization import serialize_message  # noqa: E402

from humaware_bag_evaluator.cli import evaluate_bag  # noqa: E402


# topic (namespace-relative) -> (recorded type string, message class)
TOPIC_TYPES = {
    "mode/state": ("humaware_msgs/msg/ModeState", humaware_msgs.ModeState),
    "mode/transition_state": (
        "humaware_msgs/msg/ModeTransitionState",
        humaware_msgs.ModeTransitionState,
    ),
    "safety/state": ("humaware_msgs/msg/SafetyState", humaware_msgs.SafetyState),
    "safety/mrm_state": ("humaware_msgs/msg/MRMState", humaware_msgs.MRMState),
    "locomotion/state": (
        "humaware_msgs/msg/LocomotionState",
        humaware_msgs.LocomotionState,
    ),
    "runtime/command_arbitration_state": (
        "humaware_msgs/msg/CommandArbitrationState",
        humaware_msgs.CommandArbitrationState,
    ),
    "runtime/health": ("humaware_msgs/msg/HealthState", humaware_msgs.HealthState),
    "cmd_vel/approved": ("geometry_msgs/msg/TwistStamped", geometry_msgs.TwistStamped),
}

NAMESPACE = "/mock_001/"


def _ns(seconds: float) -> int:
    return int(seconds * 1_000_000_000)


def _write_bag(bag_path: str, spans: dict) -> None:
    """Write a sqlite3 bag.

    ``spans`` maps a namespace-relative topic to the list of timestamps
    (seconds) at which to emit a default-constructed message.
    """
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    for index, topic in enumerate(spans):
        type_str, _ = TOPIC_TYPES[topic]
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=index,
                name=NAMESPACE + topic,
                type=type_str,
                serialization_format="cdr",
            )
        )
    for topic, timestamps in spans.items():
        _, message_cls = TOPIC_TYPES[topic]
        for seconds in timestamps:
            writer.write(
                NAMESPACE + topic, serialize_message(message_cls()), _ns(seconds)
            )
    # Closing the writer flushes the storage; rosbag2_py closes on delete.
    del writer


def _even_span(start: float, stop: float, step: float = 0.1) -> list:
    timestamps = []
    value = start
    while value <= stop + 1e-9:
        timestamps.append(round(value, 6))
        value += step
    return timestamps


def test_evaluate_real_namespaced_bag_reports_all_required_present():
    spans = {topic: _even_span(0.0, 2.0) for topic in TOPIC_TYPES}

    with tempfile.TemporaryDirectory() as tmp:
        bag_path = f"{tmp}/healthy_bag"
        _write_bag(bag_path, spans)
        result = evaluate_bag(bag_path)

    assert result.missing_required_topics == []
    assert result.stale_topics == []
    assert len(result.summaries) == len(TOPIC_TYPES)


def test_evaluate_real_bag_flags_stream_that_dies_mid_recording():
    spans = {topic: _even_span(0.0, 2.0) for topic in TOPIC_TYPES}
    # safety/state goes silent after 0.5s while everything else runs to 2.0s.
    spans["safety/state"] = _even_span(0.0, 0.5)

    with tempfile.TemporaryDirectory() as tmp:
        bag_path = f"{tmp}/dropout_bag"
        _write_bag(bag_path, spans)
        result = evaluate_bag(bag_path, max_gap_ns=_ns(1.0))

    # Present (so not missing) but flagged stale for the trailing silence.
    # stale_topics reports the fully-qualified name as recorded in the bag.
    assert result.missing_required_topics == []
    assert NAMESPACE + "safety/state" in result.stale_topics
    assert NAMESPACE + "cmd_vel/approved" not in result.stale_topics
