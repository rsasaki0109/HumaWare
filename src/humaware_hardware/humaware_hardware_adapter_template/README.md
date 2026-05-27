# humaware_hardware_adapter_template

Stability: experimental.

This package is a starting point for HumaWare hardware adapters. Copy
this directory, rename it (for example to
`humaware_<vendor>_<robot>_adapter`), and replace the
`_apply_to_hardware` and `_stop_hardware` stubs with vendor-specific
implementations. The template itself performs no actuation and is safe
to run in CI.

The runtime contract below is not optional. An adapter that bypasses it
is not a HumaWare adapter, regardless of what it can drive.

## Adapter responsibilities

An adapter must:

- subscribe to `cmd_vel/approved` and translate approved commands into
  vendor commands;
- publish `hardware/heartbeat` at a documented rate;
- publish or relay robot state (humanoid pose, balance, foot contact,
  joint state, IMU, battery, thermal, etc.) under the runtime namespace;
- expose adapter diagnostics with identity metadata (see below);
- stop output on runtime shutdown, MRM, E-stop, stale commands, mode
  change, or missing required state;
- include adapter identity metadata in every diagnostics message.

## Adapter forbidden behavior

An adapter must not:

- bypass `humaware_safety_manager` (do not subscribe to candidate
  command topics such as `teleop/cmd_vel`, `autonomy/cmd_vel`, or
  `policy/cmd_vel` for actuation);
- publish direct hardware commands in tests, fixtures, or examples;
- republish raw candidate commands as approved commands;
- silently fall back to an alternative command source on safety blocks;
- claim real-robot support without logs, bags, and version metadata.

## Required identity metadata

Every adapter must declare the following ROS parameters and include them
in every diagnostics message:

- `robot_model`
- `firmware_version`
- `sdk_version`
- `git_sha`
- `launch_profile`

See [`docs/adapter_checklist.md`](docs/adapter_checklist.md) for the
full pre-deployment checklist and
[`docs/verified_matrix_template.md`](docs/verified_matrix_template.md)
for the evidence format required to claim real-robot support.

## Gate behavior

The template applies the gate logic in `should_release_output`. The
adapter must keep the actuator command stream closed unless every check
passes:

- safety state is `STATE_OK` or `STATE_WARN`;
- MRM state is `STATE_NONE`;
- active mode is `TELEOP`, `AUTONOMY`, or `AI_POLICY`;
- arbitration state has been received recently and reports
  `output_enabled`;
- an approved command has been received recently.

Reasons for closure are published as the diagnostics message string so
operators can explain blocked output without reading code.

## Running the template

```bash
ros2 run humaware_hardware_adapter_template hardware_adapter_template_node \
  --ros-args -p robot_id:=mock_001 -p robot_model:=template
```

In this state the template will subscribe to runtime topics, publish a
heartbeat, and report its identity through diagnostics. It will not
drive any hardware.
