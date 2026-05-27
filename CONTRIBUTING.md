# Contributing

HumaWare is early-stage infrastructure for real humanoid robots. Contributions should improve reproducibility, safety, integration quality, or developer ergonomics.

## Development Baseline

- ROS 2 Jazzy is the default baseline.
- ROS 2 Humble can be supported where vendor SDKs require it.
- Use `colcon build` and `colcon test`.
- Keep package APIs small and documented.

## Pull Requests

Every PR should include:

- purpose and scope
- affected packages
- test commands and results
- safety impact
- migration notes for public message or API changes
- experiment card for robot or simulator behavior changes

## Experiment Card

Use this format for real robot or simulator validation:

```text
Experiment ID:
Robot:
Simulator:
Firmware:
ROS distro:
Surface:
Battery:
Network:
Git SHA:
Launch command:
Expected behavior:
Observed behavior:
Bag file:
Failure modes:
```

## Public Interfaces

Changing messages, actions, services, parameters, topic names, or external APIs requires migration notes.

## Safety

Do not bypass safety management or command arbitration. Tests must not publish directly to hardware command topics.
