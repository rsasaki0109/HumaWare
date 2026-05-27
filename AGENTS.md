@$HOME/.codex/RTK.md

# Agent Instructions

This repository is HumaWare: a ROS 2-native real-robot integration stack for humanoid robotics.

## Project Scope

- Build integration, deployment, safety, orchestration, and adapter infrastructure.
- Do not reimplement ROS 2, DDS, Nav2, MoveIt, ros2_control, Isaac ROS, MuJoCo, Gazebo, LeRobot, OpenVLA, Open-RMF, or vendor SDKs.
- Prefer connecting existing tools through stable humanoid runtime contracts.

## Direction Guardrails

- Keep HumaWare deployment-first, not simulation-first.
- Do not steer the project toward Genesis-style physics simulation, RL training, foundation model training, or benchmark-first research workflows.
- Treat Genesis, Isaac, MuJoCo, Gazebo, LeRobot, OpenVLA, and similar systems as backend simulators, AI components, or policy providers that integrate through adapters.
- Prioritize runtime orchestration, hardware abstraction, teleoperation, diagnostics, fleet management, distributed systems, rosbag-native workflows, deployment tooling, operational monitoring, reproducibility, maintainability, and extensibility.
- Prefer operational robustness for continuous real-world humanoid operation over one-off demos.
- Keep AI model outputs as candidate actions that must pass capability, mode, safety, arbitration, logging, and operator takeover boundaries.

## Safety Rules

- Never bypass `humaware_safety_manager`.
- Never publish directly to hardware command topics in tests.
- Never add E-stop bypasses, watchdog bypasses, or command-timeout bypasses.
- AI policy output is a candidate action only. It must pass mode, capability, and safety checks before execution.
- Real robot claims require logs, robot model, firmware version, environment, git SHA, launch command, expected behavior, observed behavior, and bag file when available.

## ROS 2 Rules

- Prefer standard ROS messages before adding custom messages.
- Public message changes require migration notes.
- New packages need a README and a clear stability label.
- New runtime code needs a minimal unit test or launch test unless the package is explicitly documentation-only.
- Use lifecycle, diagnostics, parameters, launch, and standard ROS conventions where practical.
- Jazzy is the baseline; Humble compatibility is optional and should be documented where needed.

## Repository Rules

- Keep core APIs stable and small.
- Keep experimental robot adapters and policy bridges isolated from core contracts.
- Do not make undocumented API, message, topic, parameter, or launch argument changes.
- Generated code must pass formatting, package lint checks where available, `colcon build`, and `colcon test`.
