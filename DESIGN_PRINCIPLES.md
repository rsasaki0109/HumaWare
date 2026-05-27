# Design Principles

## ROS 2 Native

Use ROS 2 conventions, lifecycle nodes, parameters, launch files, diagnostics, actions, services, and standard message types wherever possible.

## Real Robot First

Simulation is required, but the project optimizes for deployable real robot systems. Real-robot claims require evidence.

## Safety Gated

No planner, policy, teleop tool, script, or demo should publish directly to hardware command topics. Commands pass through mode management, safety checks, and command arbitration.

## Replace Less, Integrate Better

HumaWare does not replace Nav2, MoveIt, ros2_control, Isaac ROS, LeRobot, OpenVLA, MuJoCo, Gazebo, Open-RMF, or vendor SDKs. It integrates them behind humanoid-specific runtime boundaries.

## Stable Core, Experimental Universe

Stable runtime contracts belong in core packages. Robot-specific adapters, experimental policy bridges, and rapidly changing integrations belong in universe-style packages until proven.

## Logs Are Part of the API

A deployable robot system must be debuggable after the run. Launch commands, robot model, firmware, environment, git SHA, bags, and diagnostics are part of every serious experiment.
