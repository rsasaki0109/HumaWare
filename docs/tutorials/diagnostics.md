# Diagnostics

The diagnostics aggregator publishes:

- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)
- `runtime/health` (`humaware_msgs/msg/HealthState`)

Run the mock runtime:

```bash
ros2 launch humaware_launch mock_bringup.launch.py
```

Inspect health:

```bash
ros2 topic echo /mock_001/runtime/health
ros2 topic echo /diagnostics
```

The initial aggregator treats these topics as required:

- `mode/state`
- `safety/state`
- `safety/mrm_state`
- `locomotion/state`
- `runtime/command_arbitration_state`

Nav2 bridge and teleop heartbeat monitoring are optional because those providers may not be launched.

Runtime health is included in the default rosbag profile.
