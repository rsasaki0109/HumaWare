"""CLI entry point for ``humaware_bag_evaluator``.

This scaffold reads a rosbag2 directory and prints a summary built by
:mod:`humaware_bag_evaluator.evaluator`. The analysis logic itself lives
in pure functions to keep it testable without a bag on disk.
"""

import argparse
import sys
from typing import Iterable

from humaware_bag_evaluator.evaluator import (
    DEFAULT_REQUIRED_TOPICS,
    EvaluationResult,
    format_summary,
    summarize_topic_freshness,
)


def _read_messages(bag_path: str) -> Iterable[tuple[str, int, object]]:
    """Yield ``(topic, timestamp_ns, message)`` tuples from a bag directory.

    The import is deferred so that the analysis module remains importable
    in environments without ``rosbag2_py`` (for example, unit tests that
    only exercise the pure helpers).
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_by_topic = {
        info.name: get_message(info.type) for info in reader.get_all_topics_and_types()
    }

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        message_type = type_by_topic.get(topic)
        if message_type is None:
            continue
        try:
            message = deserialize_message(data, message_type)
        except Exception:  # pragma: no cover - degrade gracefully
            continue
        yield topic, timestamp_ns, message


def evaluate_bag(bag_path: str, max_gap_ns: int = 1_500_000_000) -> EvaluationResult:
    """Evaluate the bag at ``bag_path`` and return the structured result."""
    return summarize_topic_freshness(
        messages=_read_messages(bag_path),
        required_topics=DEFAULT_REQUIRED_TOPICS,
        max_gap_ns=max_gap_ns,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="humaware_bag_evaluator",
        description=(
            "Inspect a HumaWare runtime rosbag for required topic coverage,"
            " publication gaps, and key state transitions."
        ),
    )
    parser.add_argument("bag_path", help="Path to the rosbag2 directory.")
    parser.add_argument(
        "--max-gap-ms",
        type=float,
        default=1500.0,
        help="Maximum tolerated gap between messages on any required topic.",
    )
    args = parser.parse_args(argv)

    max_gap_ns = int(args.max_gap_ms * 1_000_000)
    result = evaluate_bag(args.bag_path, max_gap_ns=max_gap_ns)
    print(format_summary(result))

    if result.missing_required_topics or result.stale_topics:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
