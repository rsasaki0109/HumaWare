# humaware_launch

Stability: experimental.

Launch profiles for HumaWare bringup, simulation, replay, and diagnostics.

The first profile starts the mode manager, mock robot, safety manager, mock locomotion adapter, command arbiter, and diagnostics aggregator.

Keyboard teleop can be included with:

```bash
ros2 launch humaware_launch mock_bringup.launch.py enable_keyboard_teleop:=true
```

The Nav2-style velocity bridge can be included with:

```bash
ros2 launch humaware_launch mock_bringup.launch.py enable_nav2_bridge:=true
```

## Replay-only profile

`replay_only.launch.py` starts the runtime decision graph without any
hardware adapter, mock robot, or mock locomotion adapter. It is intended
for replaying rosbags through the runtime so that mode transitions,
safety state, and command arbitration can be inspected without
connecting to a real or simulated robot.

```bash
ros2 launch humaware_launch replay_only.launch.py robot_id:=mock_001
```

Then in a separate terminal, replay a runtime bag into the same
namespace:

```bash
ros2 bag play <bag_path> --remap /old_ns:=/mock_001
```

Do not replay into real hardware command adapters. The replay-only
profile intentionally omits adapters so that bag playback cannot reach
actuators or vendor SDKs.
