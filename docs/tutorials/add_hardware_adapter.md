# Add a Hardware Adapter

A hardware adapter connects vendor SDKs or ros2_control components to HumaWare runtime contracts.

## Requirements

An adapter should provide:

- robot state
- locomotion state
- safety state
- heartbeat
- command timeout handling
- stop command
- adapter README
- tested robot model and firmware matrix

## Naming

Use precise names such as:

- `humaware_unitree_h1`
- `humaware_unitree_g1_experimental`
- `humaware_generic_ros2_control`

Avoid claiming support for robot families that have not been tested.
