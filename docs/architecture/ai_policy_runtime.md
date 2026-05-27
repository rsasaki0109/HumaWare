# AI Policy Runtime

HumaWare treats AI policy output as a candidate action, not an executable robot command.

## Policy Flow

```text
PolicyProvider
  observations
  language instruction
  task context
        |
candidate action
        |
PolicySafetyGate
        |
SkillServer
        |
Command Arbiter
        |
Safety Manager
        |
Hardware or Simulator Adapter
```

## Policy Providers

Possible providers include:

- LeRobot policy
- OpenVLA policy
- reinforcement learning policy
- diffusion policy
- proprietary policy
- classical planner

## Gate Checks

The policy safety gate should check:

- mode
- body stability
- collision constraints
- speed limits
- reachability
- operator approval where required
- confidence or uncertainty
- timeout

## Initial Scope

The first implementation should expose the runtime boundary and safety checks. Full VLA integration can come later.
