# AI Policy Runtime

HumaWare treats AI policy output as a candidate action, not an executable robot command.

This runtime is an integration and operations boundary. It is not a model
training framework, an RL environment, or a foundation model repository. Policy
providers can come from LeRobot, OpenVLA, classical planners, proprietary
systems, or other model stacks, but HumaWare only accepts their outputs as
candidate actions that must pass runtime checks.

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

The first implementation should expose the runtime boundary, safety checks,
operator takeover path, logging, and replay. Full VLA integration can come
later, and model training remains outside the core scope.
