# humaware_policy_provider_stub

Stability: experimental.

A minimal policy provider for HumaWare. The stub publishes scripted
candidate velocity commands onto `policy/cmd_vel` so the rest of the
runtime (mode manager, safety manager, command arbiter) can be
exercised end-to-end without a real policy.

## Scope

The stub exists to prove the candidate-action boundary. It never:

- publishes to `cmd_vel/approved` or other approved-command topics;
- publishes to candidate topics belonging to teleop or autonomy;
- bypasses `humaware_safety_manager` or `humaware_command_arbiter`;
- runs without an explicit `enabled` parameter set to `true`;
- assumes the active mode is anything other than `MODE_AI_POLICY`.

It is not a foundation model, not a learned policy, and not an example
of how to deploy one. It is a fixture that confirms the runtime can
arbitrate, gate, and observe a candidate-action provider.

## Parameters

- `robot_id` — runtime namespace identifier (default `mock_001`).
- `enabled` — set to `true` to emit candidate commands. Defaults to
  `false`; the stub stays idle until an operator opts in.
- `publish_rate_hz` — candidate emission rate (default `10.0`).
- `linear_x_mps`, `angular_z_radps` — scripted command values.
- `provider_source` — string written into log lines, useful when
  multiple providers are running.
- `confidence` — scalar written into log lines, used to verify that
  policy provenance fields flow through later integrations.

## Gate behavior

`should_emit_candidate` returns `(allow, reason)` and is consulted on
every tick. The stub emits a candidate only when all of the following
hold:

- `enabled` parameter is `true`;
- safety state has been observed and is `STATE_OK` or `STATE_WARN`;
- active mode is `MODE_AI_POLICY`.

When the gate is closed, the stub stays silent and logs the reason at
debug level. This keeps the runtime free of noisy spurious candidates.

## Running the stub

```bash
ros2 run humaware_policy_provider_stub policy_provider_stub_node \
  --ros-args -p robot_id:=mock_001 -p enabled:=true -p linear_x_mps:=0.1
```

Before any commands flow, the mode manager must transition the runtime
into `MODE_AI_POLICY` (which itself requires entry from an active mode
such as `TELEOP`). The command arbiter then approves the stub's
candidate only while the active mode remains `MODE_AI_POLICY`.

## Tests

`test/test_gate.py` exercises the pure gate logic. Tests do not import
or run the node; they assert that the gate closes for every disqualifying
input. Real policy integrations (LeRobot, OpenVLA, vendor stacks) must
continue to honor the same gate before publishing onto `policy/cmd_vel`.
