# humaware_diagnostics_aggregator

Stability: experimental.

Aggregates HumaWare runtime state into:

- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)
- `runtime/health` (`humaware_msgs/msg/HealthState`)

Inputs:

- `mode/state`
- `mode/transition_state`
- `safety/state`
- `safety/mrm_state`
- `locomotion/state`
- `runtime/command_arbitration_state`
- `navigation/nav2_bridge_state`
- `teleop/heartbeat`

The initial aggregator treats mode, safety, locomotion, and command arbitration as required topics. Nav2 bridge and teleop heartbeat are optional by default because those providers may not be launched.
