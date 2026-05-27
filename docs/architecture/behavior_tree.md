# Behavior Tree

HumaWare uses Behavior Trees to make failure and recovery explicit.

The initial goal is not to create general intelligence. The goal is to express task execution, safety checks, fallback behavior, and operator takeover in a readable control structure.

## Example

```text
NavigateToTarget
  CheckRobotStanding
  CheckBattery
  CheckNetwork
  EnableLocomotion
  NavigateWithNav2
  OnFailure: StopAndStabilize
  OnFailure: RequestTeleop
  LogResult
```

## BT Node Categories

- mode checks
- safety checks
- capability checks
- locomotion commands
- Nav2 action wrappers
- MoveIt action wrappers
- teleop takeover
- fall recovery
- arm safe pose
- policy gate checks
- incident logging

## Design Rule

BT nodes may request capabilities. They must not bypass command arbitration or safety management.
