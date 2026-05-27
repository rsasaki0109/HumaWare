# Architecture

HumaWare is a capability-based humanoid integration runtime.

It treats a robot as a set of capabilities rather than a loose collection of topics. Capabilities can be exposed to applications, teleop tools, planners, and policy runtimes while still passing through mode checks, safety checks, and command arbitration.

The architecture is deployment-first. Physics simulators, RL policies, and
foundation models are external providers behind stable runtime interfaces. The
core system owns orchestration, safety, diagnostics, logging, replay,
distributed execution, and operational monitoring for real robots.

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

## Core Runtime

The core runtime owns:

- lifecycle orchestration
- mode management
- capability registry
- command arbitration
- safety state
- diagnostics
- incident logging

## Adapter Boundary

Adapters connect HumaWare to external systems:

- hardware adapters
- locomotion adapters
- navigation adapters
- manipulation adapters
- simulator adapters
- policy providers
- teleop providers
- fleet adapters

Adapters must preserve the same runtime contracts used by real robots. Simulator
and AI adapters must not bypass safety management, command arbitration, or
operator takeover paths.

## Command Priority

Command arbitration uses this priority order:

1. E-stop
2. Safety manager and minimal risk maneuver
3. Fall recovery
4. Teleop takeover
5. Autonomous task runtime
6. AI policy runtime
7. Demo or script command

## More Detail

See `docs/architecture/` for lifecycle, interfaces, behavior tree, distributed execution, and AI policy runtime design notes.
