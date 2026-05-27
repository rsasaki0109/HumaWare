# Teleoperation

Teleoperation is a first-class mode, not a debug shortcut.

The teleop provider should:

- publish heartbeat
- expose operator takeover state
- respect mode manager ownership
- pass commands through command arbitration
- stop on command timeout
- log operator intervention events

Teleop commands must not publish directly to hardware command topics.

Keyboard teleop publishes candidate operator velocity commands:

```bash
ros2 launch humaware_keyboard_teleop keyboard_teleop.launch.py robot_id:=mock_001
```

For the mock runtime, it can also be included in bringup:

```bash
ros2 launch humaware_launch mock_bringup.launch.py enable_keyboard_teleop:=true
```

The keyboard node must run in an interactive terminal. In non-interactive CI or launch smoke tests it stays idle and does not publish commands.

It publishes `teleop/heartbeat` even when no command key is active.

If teleop mode is active and no teleop heartbeat is seen before the configured timeout, the safety manager reports a warning by default.

The initial command path is:

```text
teleop/cmd_vel
  |
humaware_command_arbiter
  |
cmd_vel/approved
  |
humaware_locomotion_adapter
  |
locomotion/state
```

The arbiter approves teleop velocity only when the active mode is `MODE_TELEOP` and safety state allows output.

Operator takeover from autonomy or AI policy mode uses:

```bash
ros2 service call /mock_001/mode/takeover humaware_msgs/srv/Takeover "{requester: operator, reason: operator_takeover}"
```
