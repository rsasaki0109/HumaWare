# Add a Safety Policy

Safety policies are plugins or modules that evaluate candidate actions and runtime state.

## Policy Inputs

- mode
- command source
- requested capability
- robot state
- balance state
- locomotion state
- battery and thermal state
- network heartbeat
- operator approval

## Policy Outputs

- allow
- reject
- limit
- request teleop
- trigger minimal risk maneuver

Policies must be deterministic, logged, and testable.
