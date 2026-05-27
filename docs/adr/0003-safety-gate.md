# ADR 0003: Safety Gate

## Status

Accepted.

## Decision

All executable robot commands pass through command arbitration and safety management before reaching hardware or simulator adapters.

## Consequences

- AI policy output is a candidate action only.
- Teleop is high priority but still subject to safety stop.
- Tests must not publish directly to hardware command topics.
- Hardware adapters must expose stop and timeout behavior.
