# humaware_locomotion_interface

Stability: experimental.

This package documents the first HumaWare locomotion adapter contract. It intentionally uses standard ROS messages for velocity commands and HumaWare messages for runtime state.

## Contract

Input:

- `cmd_vel/approved` (`geometry_msgs/msg/TwistStamped`)

Output:

- `locomotion/state` (`humaware_msgs/msg/LocomotionState`)

State dependencies:

- `mode/state` (`humaware_msgs/msg/ModeState`)
- `safety/state` (`humaware_msgs/msg/SafetyState`)

## Adapter Responsibilities

A locomotion adapter must:

- consume only approved commands, not raw teleop, Nav2, or policy commands
- publish `locomotion/state`
- stop on command timeout
- stop on unsafe safety state
- respect inactive, maintenance, fault, and shutdown modes
- expose its adapter name in `active_adapter`
- report applied velocity limits

## Initial Capabilities

The first contract covers:

- `stand`
- `stop`
- `walk_velocity`
- `turn_in_place`
- `recover_posture`

Robot-specific gait generation remains behind the adapter boundary.
