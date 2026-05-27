# humaware_mode_manager

Stability: experimental.

Publishes the current HumaWare runtime mode and accepts mode transition requests through `humaware_msgs/srv/SetMode`.

The mode manager validates transition requests against safety state and runtime guard rules. It also exposes operator takeover through `mode/takeover`.

Services:

- `mode/set` (`humaware_msgs/srv/SetMode`)
- `mode/takeover` (`humaware_msgs/srv/Takeover`)

Published state:

- `mode/state` (`humaware_msgs/msg/ModeState`)
- `mode/transition_state` (`humaware_msgs/msg/ModeTransitionState`)

Initial guard rules:

- active modes require a known safety state
- autonomy and AI policy are blocked during fault, E-stop, or MRM
- maintenance cannot jump directly to autonomy or AI policy
- shutdown is terminal
- operator takeover can switch autonomy or AI policy to teleop
