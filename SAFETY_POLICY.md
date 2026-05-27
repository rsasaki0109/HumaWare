# Safety Policy

HumaWare safety is operational safety for real robot integration. This repository does not claim certification against safety standards.

## Non-Negotiable Rules

- Never bypass `humaware_safety_manager`.
- Never publish directly to hardware command topics in tests.
- Never add code that disables E-stop, watchdogs, command timeouts, or minimal risk behavior.
- Never treat AI policy output as executable hardware command output.
- Real robot test claims require logs, robot model, firmware version, environment, git SHA, and expected versus observed behavior.

## Minimum Safety Features

The runtime must support:

- E-stop integration
- command timeout
- heartbeat
- network loss behavior
- battery, thermal, CPU, and memory monitoring
- fall detection
- unsafe posture detection
- velocity and acceleration limits
- arm safe pose
- autonomous, teleop, maintenance, and fault mode separation
- minimal risk maneuver
- request-to-intervene
- rosbag logging for incident replay

## Command Flow

Commands from teleop, Nav2, MoveIt, AI policies, demos, and scripts must pass through command arbitration and safety checks before reaching hardware adapters.
