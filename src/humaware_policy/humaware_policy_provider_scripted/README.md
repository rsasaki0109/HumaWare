# humaware_policy_provider_scripted

Stability: experimental.

A scripted HumaWare policy provider. Reads a waypoint plan from YAML
and publishes candidate velocity commands onto `policy/cmd_vel`. It is
intended as a small, deterministic example of a candidate-action
provider — useful for runtime integration tests, fixture bags, and as
an authoring reference for richer providers (LeRobot, OpenVLA, vendor
stacks) that arrive later.

## Scope

The provider:

- emits `geometry_msgs/msg/TwistStamped` candidates onto `policy/cmd_vel`;
- walks a static, hand-authored waypoint plan in order;
- honors the same candidate-action gate as
  `humaware_policy_provider_stub` — see "Gate behavior" below;
- pauses progress whenever the gate is closed, so a brief mode or
  safety excursion does not silently skip waypoints.

The provider never:

- publishes to `cmd_vel/approved` or any actuator topic;
- publishes to candidate topics belonging to teleop or autonomy;
- bypasses `humaware_safety_manager` or `humaware_command_arbiter`;
- runs without an explicit `enabled` parameter set to `true`;
- assumes the active mode is anything other than `MODE_AI_POLICY`.

## Plan format

```yaml
loop: false
waypoints:
  - linear_x_mps: 0.05
    angular_z_radps: 0.0
    duration_s: 2.0
  - linear_x_mps: 0.0
    angular_z_radps: 0.2
    duration_s: 1.5
```

- `loop` — when `true`, the provider restarts from the first waypoint
  after the last one completes. When `false` (the default), the
  provider stays idle once the plan completes.
- `waypoints` — ordered list of `(linear_x_mps, angular_z_radps,
  duration_s)`. Each `duration_s` must be strictly positive.

Keep velocities inside the arbiter's clamp limits — the arbiter is the
authority on motion limits, not this provider.

## Parameters

- `robot_id` — runtime namespace identifier (default `mock_001`).
- `enabled` — set to `true` to emit candidate commands. Defaults to
  `false`; the provider stays idle until an operator opts in.
- `plan_yaml_path` — absolute path to a plan YAML file. If empty or
  unreadable the provider stays idle.
- `publish_rate_hz` — tick rate (default `10.0`).
- `provider_source` — string written into log lines.
- `confidence` — scalar written into log lines, used to verify that
  policy provenance fields flow through later integrations.

## Gate behavior

`should_emit_candidate` returns `(allow, reason)` and is consulted on
every tick. The provider only emits when:

- `enabled` parameter is `true`;
- safety state has been observed and is `STATE_OK` or `STATE_WARN`;
- active mode is `MODE_AI_POLICY`.

When the gate is closed the provider stays silent **and does not
advance the waypoint plan**, so a brief excursion (e.g. a momentary
safety `WARN`) resumes the plan from the same position rather than
skipping ahead.

## Running

```bash
ros2 run humaware_policy_provider_scripted policy_provider_scripted_node \
  --ros-args \
  -p robot_id:=mock_001 \
  -p enabled:=true \
  -p plan_yaml_path:=$(ros2 pkg prefix humaware_policy_provider_scripted)/share/humaware_policy_provider_scripted/config/example_plan.yaml
```

Before any commands flow, the mode manager must transition the runtime
into `MODE_AI_POLICY` (which itself requires entry from an active mode
such as `TELEOP`). The command arbiter then approves the scripted
provider's candidate only while the active mode remains
`MODE_AI_POLICY`.

## Tests

- `test/test_plan.py` — pure waypoint state machine: parse, advance,
  wrap, complete, idle behavior on the empty plan.
- `test/test_gate.py` — the candidate-action gate. Real policy
  integrations must honor the same gate before publishing onto
  `policy/cmd_vel`.

The tests do not import or run the node; they assert on pure helpers.

## Non-goals

- Learned or model-driven policies — see future LeRobot/OpenVLA bridges.
- Closed-loop sensor feedback — the plan is open-loop by design.
- Dynamic re-planning — re-load the node with a new YAML to change the
  plan.
