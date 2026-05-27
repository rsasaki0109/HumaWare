# Nav2 Integration

HumaWare uses Nav2 instead of replacing it.

The Nav2 bridge translates navigation intent into humanoid locomotion capabilities:

- stand
- walk velocity
- turn in place
- stop
- recover posture

The bridge must account for humanoid-specific state:

- balance state
- foot contact
- fall risk
- posture
- locomotion readiness
- arm safe pose policy

Nav2 output is a candidate movement request. The command arbiter and safety manager decide whether it can be executed.

The initial Nav2-style command path uses `autonomy/cmd_vel` as the candidate input and `cmd_vel/approved` as the gated output.

The locomotion adapter is responsible for translating `cmd_vel/approved` into humanoid locomotion state and, in a real adapter, vendor gait commands.

The initial bridge accepts Nav2-style velocity commands:

```text
nav2/cmd_vel
  |
humaware_nav2_bridge
  |
autonomy/cmd_vel
  |
humaware_command_arbiter
  |
cmd_vel/approved
  |
humaware_locomotion_adapter
  |
locomotion/state
```

Run the mock bridge:

```bash
ros2 launch humaware_launch mock_bringup.launch.py enable_nav2_bridge:=true
```

Then set autonomy mode and publish a test command:

```bash
ros2 service call /mock_001/mode/set humaware_msgs/srv/SetMode "{requested_mode: 4, requester: nav2_test, reason: nav2_bridge}"
ros2 topic pub -r 10 /mock_001/nav2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}"
```
