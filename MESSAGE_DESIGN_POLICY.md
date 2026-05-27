# Message Design Policy

Custom messages are a public contract. Add them slowly.

## Rules

- Prefer standard ROS messages first.
- Use `std_msgs/Header` for timestamped state.
- Use `geometry_msgs`, `nav_msgs`, `trajectory_msgs`, `control_msgs`, `diagnostic_msgs`, and `sensor_msgs` where they fit.
- Do not encode structured data in strings when a typed field is practical.
- Keep command messages separate from state messages.
- Never change public messages without migration notes.
- Include units in field names where ambiguity is possible.
- Prefer capability-level messages over joint-level abstractions for public APIs.

## Initial Message Set

The initial humanoid-specific runtime messages are:

- `HumanoidState`
- `LocomotionState`
- `BalanceState`
- `FootContactState`
- `SafetyState`
- `Capability`
- `ModeState`
- `MRMState`

These messages describe runtime state and capability contracts. They are not a low-level whole-body control API.
