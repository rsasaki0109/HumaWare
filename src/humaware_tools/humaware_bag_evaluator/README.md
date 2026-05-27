# humaware_bag_evaluator

Stability: experimental.

Scaffold for evaluating HumaWare runtime rosbags. Given a bag recorded
by `humaware_bag_profiles record_runtime.launch.py`, the tool reports:

- which required runtime topics are missing;
- which topics exceed the configured publication gap threshold;
- counts and longest gaps per topic;
- mode, safety, MRM, command-arbitration, and locomotion state
  transitions.

It is intentionally read-only and does not interact with hardware
adapters or runtime services.

## Usage

```bash
ros2 run humaware_bag_evaluator humaware_bag_evaluator <bag_path>
```

Optional arguments:

- `--max-gap-ms` — the largest gap (in milliseconds) tolerated between
  consecutive messages on any topic. Topics that exceed the gap are
  reported as stale (default `1500`).

The tool exits with a non-zero status if any required topic is missing
or any topic exceeded the gap threshold. Use this for CI hooks against
fixture bags once those are introduced.

## Layout

- `humaware_bag_evaluator/evaluator.py` — pure-function helpers
  (`summarize_topic_freshness`, `format_summary`, `EvaluationResult`).
  Tests target this module so the rules can be exercised without a bag.
- `humaware_bag_evaluator/cli.py` — bag reader and command-line entry
  point. Wraps `rosbag2_py` and feeds messages into the pure helpers.

## Non-goals

- analysing bags from non-HumaWare projects;
- inspecting actuator state — actuator topics belong to adapters and
  must not be required by this tool;
- replaying bags. Use `humaware_launch replay_only.launch.py` for that.
