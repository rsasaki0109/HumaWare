# Manifesto

HumaWare exists because humanoid robotics has a gap between research code and real-world deployable systems.

The project is not a middleware replacement, a walking-controller project, a universal humanoid brain, or an end-to-end AI robot OS. It is a ROS 2-native integration and deployment stack for real humanoid robots.

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
- simulator-to-real parity
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
- LeRobot or OpenVLA
- vendor low-level SDKs

HumaWare connects these systems safely and reproducibly for humanoid robots.

## Core Promise

From ROS 2 packages to deployable humanoid systems.
