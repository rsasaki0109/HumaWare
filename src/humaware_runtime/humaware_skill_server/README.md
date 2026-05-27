# humaware_skill_server

Stability: experimental.

Accepts capability execution requests through a ROS service and emits only
candidate commands that still pass through command arbitration and safety.

Default service:

```text
/<robot_id>/skills/execute
```

Default state topic:

```text
/<robot_id>/skills/state
```

Initial executable skills:

- `stop`
- `walk_velocity`
- `turn_in_place`

Unsupported capabilities are rejected explicitly instead of being silently
mapped to low-level robot commands.
