# humaware_nav2_bridge

Stability: experimental.

Bridge from Nav2-style velocity commands to HumaWare autonomy candidate commands.

Input:

- `nav2/cmd_vel` (`geometry_msgs/msg/Twist`)
- `nav2/cmd_vel_stamped` (`geometry_msgs/msg/TwistStamped`)

Output:

- `autonomy/cmd_vel` (`geometry_msgs/msg/TwistStamped`)
- `navigation/nav2_bridge_state` (`humaware_msgs/msg/NavigationBridgeState`)

The bridge does not publish hardware commands. It publishes autonomy candidate commands, which must still pass through `humaware_command_arbiter` and safety management before reaching a locomotion adapter.

## Run

```bash
ros2 launch humaware_launch mock_bringup.launch.py enable_nav2_bridge:=true
```
