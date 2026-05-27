# Safety Architecture

HumaWare safety is practical real-robot operating safety. It is not a certification claim.

## Safety Boundary

All executable robot commands must cross the safety boundary:

```text
Teleop / Nav2 / MoveIt / AI Policy / Script
        |
Command Arbiter
        |
Safety Manager
        |
Hardware or Simulator Adapter
```

Mode transitions are also guarded. Autonomy and AI policy modes are blocked when safety state is unknown, faulted, E-stopped, or in MRM.

The initial safety manager monitors:

- `teleop/heartbeat`
- `hardware/heartbeat`
- `cmd_vel/approved`
- `mode/state`

It exposes:

- `safety/trigger_mrm`
- `safety/clear_mrm`
- `safety/state`
- `safety/mrm_state`

Runtime health and diagnostics are aggregated separately by `humaware_diagnostics_aggregator`.

## Minimum Checks

The safety manager should check:

- E-stop
- command timeout
- heartbeat
- network loss
- battery state
- thermal state
- CPU and memory pressure
- fall state
- unsafe posture
- velocity limits
- acceleration limits
- arm safe pose
- mode permission

## Minimal Risk Maneuver

The initial MRM set should include:

- stop locomotion
- stabilize posture
- sit or crouch where supported
- hold arm safe pose
- request teleop
- log incident data

MRM behavior must be local to the robot or operator station. Cloud systems must not be required for safety stop or stabilization.
