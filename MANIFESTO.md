# Manifesto

HumaWare exists because humanoid robotics has a gap between research code and real-world deployable systems.

The project is not a middleware replacement, a walking-controller project, a universal humanoid brain, or an end-to-end AI robot OS. It is a ROS 2-native integration and deployment stack for real humanoid robots.

## Direction

HumaWare is not a physics simulation project, an RL project, a foundation model
training project, or a benchmark-first research repo.

The project exists to operate real humanoid robots as distributed ROS 2 systems.
Simulation engines, robot learning libraries, and foundation models can plug in
as backends or providers, but they do not define the core architecture.

We choose:

- deployment-first over simulation-first
- operational robustness over benchmarks
- continuous operation over one-off demos
- modular integration over monolithic AI
- distributed robotics over single-agent assumptions
- reusable infrastructure over research scripts

## What We Build

We build the boring infrastructure humanoids need:

- bringup and shutdown
- lifecycle orchestration
- safety gates
- command arbitration
- teleop and autonomy handoff
- locomotion and Nav2 integration
- manipulation integration boundaries
- diagnostics
- rosbag replay
- simulator adapter parity with real runtime interfaces
- hardware adapter templates
- AI policy runtime boundaries

## What We Do Not Build First

The project does not begin by competing with:

- ROS 2
- DDS or Zenoh
- Nav2
- MoveIt
- ros2_control
- Isaac ROS
- MuJoCo or Gazebo
- Genesis
- LeRobot or OpenVLA
- vendor low-level SDKs

HumaWare connects these systems safely and reproducibly for humanoid robots.

## Core Promise

From ROS 2 packages to deployable humanoid systems.
