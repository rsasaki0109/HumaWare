# Architecture Overview

HumaWare is a capability-based runtime for integrating humanoid robot systems.

The runtime sits above ROS 2, vendor SDKs, simulators, and domain stacks such as Nav2 and MoveIt. It provides the boring but essential layer needed to bring up, monitor, stop, replay, and extend a real humanoid robot.

HumaWare is not centered on simulation, RL, or model training. Those systems are
integrated as providers or adapters. The core design goal is to keep real
humanoid robots running, observable, recoverable, and reproducible in continuous
operation.

```text
Application / Fleet / Teleop / AI Policy
        |
HumaWare Skill API / External API
        |
Task Runtime / Behavior Tree / Mode Manager
        |
Safety Gate / Capability Arbiter
        |
Locomotion / Navigation / Manipulation / Perception Servers
        |
Hardware & Simulator Adapters
        |
ROS 2 / ros2_control / vendor SDK / DDS / sensors / actuators
```

## Stable Core

Core packages own stable runtime contracts:

- lifecycle
- safety state
- diagnostics
- mode management
- capability registry
- command arbitration
- incident logging

## Universe-Style Integrations

Experimental packages and integrations should remain outside the stable core until validated:

- robot-specific adapters
- AI policy bridges
- simulator-specific backend profiles
- perception acceleration profiles
- fleet adapters

## First MVP

The first MVP is one humanoid robot, real or mock, that can:

- start from launch
- expose runtime state
- switch between inactive, teleop, autonomy, and fault modes
- accept teleop and Nav2-style movement through safe boundaries
- stop on timeout or safety fault
- record and replay logs

## Initial Runtime Topics

The mock runtime starts with these relative topics under the robot namespace:

- `mode/state`
- `mode/set`
- `mode/takeover`
- `mode/transition_state`
- `safety/state`
- `safety/mrm_state`
- `safety/trigger_mrm`
- `safety/clear_mrm`
- `teleop/cmd_vel`
- `teleop/heartbeat`
- `nav2/cmd_vel`
- `nav2/cmd_vel_stamped`
- `autonomy/cmd_vel`
- `policy/cmd_vel`
- `cmd_vel/approved`
- `locomotion/state`
- `navigation/nav2_bridge_state`
- `runtime/command_arbitration_state`
- `runtime/health`
- `/diagnostics`
