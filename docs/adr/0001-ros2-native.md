# ADR 0001: ROS 2 Native

## Status

Accepted.

## Decision

HumaWare is ROS 2-native and builds on ROS 2 conventions, lifecycle, diagnostics, launch, parameters, actions, services, messages, and DDS communication.

## Consequences

- HumaWare does not replace ROS 2 or DDS.
- ROS 2 Jazzy is the initial baseline.
- Humble support can exist for vendor SDK compatibility.
- Standard ROS messages are preferred before custom messages.
