# humaware_bag_profiles

Stability: experimental.

Rosbag record and replay profiles for HumaWare runtime topics.

This package records runtime state, safety state, diagnostics, and candidate or approved command topics. Replay profiles are for incident analysis and must not be used while hardware adapters are connected to command topics.

## Record

```bash
ros2 launch humaware_bag_profiles record_runtime.launch.py robot_id:=mock_001
```

## Replay

```bash
ros2 launch humaware_bag_profiles replay_runtime.launch.py bag_path:=artifacts/bags/<bag>
```

## Safety

Do not replay runtime bags into a ROS graph connected to real hardware command adapters. Use an isolated ROS domain, offline workstation, or replay-only launch profile.
