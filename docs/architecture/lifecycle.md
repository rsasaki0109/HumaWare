# Lifecycle

HumaWare uses lifecycle orchestration as a first-class runtime concept.

ROS 2 lifecycle nodes provide managed states such as unconfigured, inactive, and active. HumaWare extends that idea from individual nodes to the whole humanoid system.

## Runtime Phases

```text
Unconfigured
  |
SensorsConfigured
  |
KinematicsConfigured
  |
LocomotionReady
  |
TeleopReady
  |
AutonomyReady
  |
Active
  |
Fault / MRM / Shutdown
```

## Responsibilities

Lifecycle orchestration must coordinate:

- sensors
- state estimation
- TF
- locomotion adapter
- teleop provider
- Nav2 bridge
- safety manager
- diagnostics
- behavior tree runtime
- bag recording profile

## Shutdown

Shutdown should occur in the reverse order of activation where practical:

1. stop executable commands
2. enter minimal risk maneuver if needed
3. deactivate autonomy and teleop
4. deactivate locomotion
5. stop bag recording
6. stop sensors and diagnostics
