# humaware_keyboard_teleop

Stability: experimental.

Keyboard teleoperation provider for HumaWare.

This package publishes candidate operator commands to `teleop/cmd_vel` using `geometry_msgs/msg/TwistStamped`. The command arbiter decides whether these commands can become `cmd_vel/approved`.

It also publishes `teleop/heartbeat` using `std_msgs/msg/Header`.

## Keys

- `w`: increase forward velocity
- `s`: decrease forward velocity
- `a`: turn left
- `d`: turn right
- `x` or space: stop
- `h` or `?`: print help

## Run

```bash
ros2 launch humaware_keyboard_teleop keyboard_teleop.launch.py robot_id:=mock_001
```

Or include it in mock bringup:

```bash
ros2 launch humaware_launch mock_bringup.launch.py enable_keyboard_teleop:=true
```

The terminal running this node must be interactive.
