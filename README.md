# HumaWare

HumaWare is a ROS 2-native real-robot integration stack for humanoid robotics.

It focuses on deployment, safety, orchestration, hardware abstraction, and embodied AI integration. It does not replace ROS 2, Nav2, MoveIt, ros2_control, Isaac ROS, MuJoCo, LeRobot, OpenVLA, or vendor SDKs.

HumaWare turns research-grade humanoid code into deployable real-robot systems.

## Positioning

HumaWare is the open integration runtime for real-world humanoid robots:

- ROS 2-native
- real-robot-first
- safety-gated
- policy-ready
- fleet-aware
- deployment-oriented

The first target is not a new walking controller or a new VLA model. The first target is a reproducible runtime where one humanoid robot can be brought up under ROS 2 lifecycle control, switched between teleoperation and Nav2-driven movement, stopped safely on faults, and replayed from logs.

## Direction Guardrails

HumaWare is deployment-first infrastructure. It is not a Genesis-style physics
simulation platform, an RL training environment, a foundation model training
codebase, or a benchmark-first research repository.

Simulators, robot learning libraries, and AI models are backend components that
can be integrated through adapters and policy providers. They are not the center
of the project. The center of the project is continuous real-world operation:
bringup, mode management, teleoperation, safety, diagnostics, logging, replay,
deployment tooling, fleet integration, and operational monitoring.

The project optimizes for reusable infrastructure that keeps real humanoid
systems running, debuggable, and recoverable over time.

## Scope

HumaWare provides the integration layer between application logic, autonomy stacks, robot learning policies, hardware adapters, simulators, and real robot safety systems.

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

## MVP

The initial MVP is:

> One real humanoid robot starts under ROS 2 lifecycle management, switches between teleop and Nav2 goal movement, enters a minimal risk state on abnormal conditions, and can reproduce the same configuration in simulation or rosbag replay.

The first implementation areas are:

- `humaware_msgs`
- `humaware_safety_manager`
- `humaware_mode_manager`
- `humaware_capability_registry`
- `humaware_skill_server`
- `humaware_command_arbiter`
- `humaware_locomotion_interface`
- `humaware_mock_locomotion_adapter`
- `humaware_teleop`
- `humaware_keyboard_teleop`
- `humaware_nav2_bridge`
- `humaware_bringup`
- `humaware_diagnostics`
- `humaware_diagnostics_aggregator`
- `humaware_bag_profiles`
- `humaware_mock_robot`
- backend simulator and rosbag replay profiles

## Repository Status

This repository is in foundation stage. The current tree contains the project policy documents, ROS 2 interface package skeletons, launch scaffolding, and development infrastructure needed to begin implementation.

## Quick Start

HumaWare targets ROS 2 Jazzy as the baseline. Humble can be supported where vendor SDKs require it.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch humaware_launch mock_bringup.launch.py
```

If ROS 2 is not installed locally, use the devcontainer or Docker image under `containers/`.

## Design Rules

- Prefer standard ROS messages before adding custom messages.
- Never send unvalidated AI, planner, or teleop commands directly to hardware topics.
- Route executable robot commands through safety management and command arbitration.
- Treat simulation as an adapter for runtime validation, not as the primary product.
- Prefer operational robustness over one-off demos or leaderboard-style benchmarks.
- Treat real-robot claims as experimental results: include robot model, firmware, environment, logs, bags, and git SHA.
- Keep the core stable and move experimental adapters into universe-style packages.

## Documentation

- [Manifesto](MANIFESTO.md)
- [Architecture](ARCHITECTURE.md)
- [Roadmap](ROADMAP.md)
- [Safety Policy](SAFETY_POLICY.md)
- [Message Design Policy](MESSAGE_DESIGN_POLICY.md)
- [Package Naming](PACKAGE_NAMING.md)
- [Docs Index](docs/index.md)

## License

Apache License 2.0. See [LICENSE](LICENSE).
