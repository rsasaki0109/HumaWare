# Distributed Execution

Humanoid robot systems are usually distributed across the robot, operator station, edge servers, and cloud systems.

## Runtime Placement

Robot onboard computer:

- motor and vendor SDK bridge
- local state
- safety stop
- posture stabilization
- local perception needed for safety

Operator station:

- RViz or Foxglove
- teleop
- debugging
- request-to-intervene

Edge server:

- map services
- heavy perception
- logging
- VLA or policy inference when latency allows

Cloud:

- dataset storage
- model registry
- fleet dashboard
- long-term logs
- non-critical planning

## Safety Placement

Safety stop and posture stabilization must not depend on cloud availability.

## Network Strategy

Use ROS 2 and DDS by default. Use Zenoh bridge profiles for remote, low-bandwidth, or NAT-crossing deployments when ROS graph bridging is needed.
