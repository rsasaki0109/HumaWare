# humaware_command_arbiter

Stability: experimental.

Arbitrates candidate velocity commands from teleop, autonomy, and AI policy sources.

Input topics are relative to the robot namespace:

- `teleop/cmd_vel`
- `autonomy/cmd_vel`
- `policy/cmd_vel`
- `mode/state`
- `safety/state`

Output topics:

- `cmd_vel/approved`
- `runtime/command_arbitration_state`

The arbiter publishes approved velocity commands only when the active mode, source priority, command freshness, and safety state allow it.
