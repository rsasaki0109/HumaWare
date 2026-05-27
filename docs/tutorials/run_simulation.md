# Run Simulation

The first simulation profile is a mock bringup path. It validates launch structure, runtime state topics, and safety-manager wiring before a full Gazebo or MuJoCo scenario exists.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch humaware_launch mock_bringup.launch.py
```

Future simulation profiles should preserve the same runtime interfaces used by real robots.

The initial mock launch starts:

- mode manager
- mock robot state publisher
- safety manager
- mock locomotion adapter
- command arbiter

Keyboard teleop can be included when running from an interactive terminal:

```bash
ros2 launch humaware_launch mock_bringup.launch.py enable_keyboard_teleop:=true
```

The Nav2 bridge can be included without running Nav2 itself:

```bash
ros2 launch humaware_launch mock_bringup.launch.py enable_nav2_bridge:=true
```
