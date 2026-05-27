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
