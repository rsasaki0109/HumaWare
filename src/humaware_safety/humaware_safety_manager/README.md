# humaware_safety_manager

Stability: experimental.

This package owns the first safety state and minimal risk maneuver boundary.

It publishes safety state and MRM state so launch files, diagnostics, and downstream adapters can bind to stable topic names while safety logic evolves.

Inputs:

- `mode/state`
- `teleop/heartbeat`
- `hardware/heartbeat`
- `cmd_vel/approved`

Services:

- `safety/trigger_mrm`
- `safety/clear_mrm`

Default watchdog behavior:

- missing or stale teleop heartbeat in teleop mode: warning
- missing or stale hardware heartbeat when required: MRM
- stale approved command after commands have started: warning

Watchdog warnings can be promoted to MRM with parameters.
