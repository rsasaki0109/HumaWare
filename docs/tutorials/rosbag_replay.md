# Rosbag Replay

Rosbag replay is required for incident analysis and reproducibility.

Every real robot experiment should record:

- runtime state
- safety state
- mode state
- diagnostics
- TF
- odometry
- command requests
- command approvals or rejections
- operator takeover events

Replay profiles should not publish to hardware command topics.

## Record Runtime Bag

Run the mock runtime in one terminal:

```bash
ros2 launch humaware_launch mock_bringup.launch.py
```

Record the runtime profile in another terminal:

```bash
ros2 launch humaware_bag_profiles record_runtime.launch.py robot_id:=mock_001
```

Or use the helper:

```bash
scripts/record_runtime_bag.sh
```

The default output is:

```text
artifacts/bags/<robot_id>_<utc_timestamp>
```

## Replay Runtime Bag

Replay on an offline graph:

```bash
ros2 launch humaware_bag_profiles replay_runtime.launch.py bag_path:=artifacts/bags/<bag>
```

Do not run replay while real hardware adapters are connected to command topics.

## Default Runtime Topics

The default profile records:

- `mode/state`
- `mode/transition_state`
- `safety/state`
- `safety/mrm_state`
- `runtime/health`
- `locomotion/state`
- `runtime/command_arbitration_state`
- `navigation/nav2_bridge_state`
- `teleop/heartbeat`
- `cmd_vel/approved`
- candidate command topics
- `/diagnostics`
- `/tf`
- `/tf_static`
- `/clock`
