# Interfaces

HumaWare exposes humanoid systems as capabilities.

## Capability Model

A capability describes what a robot can do and the runtime conditions required to do it.

Fields include:

- name
- state
- required mode
- required hardware
- input type
- output type
- safety constraints
- timeout
- recovery behavior
- owner node

## Initial Capabilities

- `stand`
- `sit`
- `walk_velocity`
- `walk_to_pose`
- `turn_in_place`
- `stop`
- `recover_posture`
- `move_arm`
- `grasp`
- `look_at`
- `speak`
- `request_teleop`
- `dock`
- `inspect`

## Command Boundary

Applications and policies should call capabilities or stable external APIs. They should not publish directly to hardware command topics.

Initial velocity command candidates use standard `geometry_msgs/TwistStamped` messages on source-specific topics. The command arbiter publishes only approved commands to `cmd_vel/approved`.

The locomotion adapter consumes `cmd_vel/approved` and publishes `locomotion/state`. Robot-specific gait control, vendor SDK calls, and ros2_control bindings stay behind that adapter boundary.

Mode transitions go through `mode/set`; operator takeover goes through `mode/takeover`. Rejections are visible on `mode/transition_state`.

## External API

External APIs should be stable and narrow. Fleet systems, operator consoles, and applications should depend on `humaware_adapi_msgs` once that package is introduced.
