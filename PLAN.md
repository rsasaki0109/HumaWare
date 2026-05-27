# HumaWare Execution Plan

This document is the working implementation plan for HumaWare. `ROADMAP.md`
summarizes the phases. This file explains what to build, in what order, what to
avoid, and how to judge whether the system is becoming useful as real-world
humanoid infrastructure.

## North Star

HumaWare is a ROS 2-native real-robot integration runtime for humanoids.

The project should turn research-grade humanoid code, vendor SDKs, teleop tools,
Nav2, MoveIt, perception stacks, policy providers, and operational tooling into
a deployable real-robot system. The value is not in replacing those systems. The
value is in making them start together, expose stable runtime state, arbitrate
commands, stop safely, record evidence, replay incidents, and remain extensible.

The practical target is:

> A real humanoid can be brought up repeatedly, operated by a human, handed to
> autonomy or a policy runtime, stopped on faults, inspected through diagnostics,
> and reproduced from logs without relying on ad hoc launch scripts.

## Non-Goals

HumaWare must not drift into a Genesis-style simulator project, an RL training
environment, a foundation model training repository, or a benchmark-first
research project.

The project does not own:

- physics engine development
- RL locomotion training
- foundation model training
- VLA model architecture
- whole-body control research
- vendor firmware
- Nav2, MoveIt, ros2_control, Open-RMF, Isaac ROS, MuJoCo, Gazebo, LeRobot, or
  OpenVLA replacements
- leaderboard-style benchmark competition as the primary product

Those systems can integrate through adapters and providers. HumaWare owns the
runtime boundary around them.

## Design Priorities

The priority order is:

1. Real-robot operational robustness
2. Safety gating and command arbitration
3. ROS 2-native interfaces and tooling
4. Reproducible bringup, shutdown, logging, and replay
5. Teleop and operator takeover
6. Hardware and simulator adapter contracts
7. Distributed execution across robot, operator station, edge, and cloud
8. Fleet-aware naming and integration boundaries
9. AI policy integration as candidate-action providers
10. Developer ergonomics and contribution quality

When tradeoffs appear, choose the option that makes a real robot easier to
operate, debug, and recover.

## Runtime Architecture

The stable runtime shape is:

```text
Application / Fleet / Operator / AI Policy
        |
Skill API / External API
        |
Task Runtime / Behavior Tree / Mode Manager
        |
Safety Gate / Capability Registry / Command Arbiter
        |
Locomotion / Navigation / Manipulation / Perception Boundaries
        |
Hardware Adapters / Simulator Adapters / Policy Providers
        |
ROS 2 / ros2_control / vendor SDK / DDS / sensors / actuators
```

The key rule is that no planner, policy, teleop node, script, or demo publishes
directly to hardware command topics. Candidate commands go through runtime mode,
capability, safety, and arbitration boundaries before they become approved robot
commands.

## Core Runtime Contracts

The initial stable contract should converge around these packages:

- `humaware_msgs`: runtime messages and service contracts
- `humaware_mode_manager`: authoritative runtime mode ownership
- `humaware_safety_manager`: safety state, MRM state, watchdogs, and stop logic
- `humaware_capability_registry`: machine-readable capability availability
- `humaware_skill_server`: capability-oriented execution API
- `humaware_command_arbiter`: candidate command selection and approval
- `humaware_locomotion_interface`: locomotion adapter contract
- `humaware_diagnostics_aggregator`: runtime health and standard diagnostics
- `humaware_bag_profiles`: rosbag record and replay profiles
- `humaware_launch`: repeatable bringup profiles

Experimental integrations should remain outside the stable contract until they
are tested and documented.

## Phase 0: Foundation Hardening

Goal: make the repository difficult to misinterpret and easy to build.

Tasks:

- Keep README, manifesto, architecture, safety policy, roadmap, and plan aligned.
- Keep `AGENTS.md` explicit about deployment-first direction.
- Keep package names, topic names, and message names consistently `humaware_*`.
- Keep CI green on ROS 2 Jazzy.
- Keep a clean mock bringup path for runtime integration tests.
- Prefer small, testable runtime rules over broad abstractions.
- Add migration notes whenever public messages or services change.

Exit criteria:

- A new contributor can explain what HumaWare is not.
- `colcon build` and `colcon test` pass in CI.
- The mock runtime starts and publishes mode, safety, locomotion, diagnostics,
  capabilities, skill state, arbitration state, and health state.

## Phase 1: Runtime MVP

Goal: one mock or real humanoid runtime can be operated through a safe ROS 2
control boundary.

Required behavior:

- Start from launch in inactive mode.
- Publish mode, safety, locomotion, capability, skill, command arbitration, and
  health topics.
- Switch into teleop only after safety state is known.
- Switch into AI policy only from an already active runtime mode such as teleop
  or autonomy.
- Accept candidate teleop, autonomy, and policy commands.
- Approve only commands whose source matches the active mode.
- Publish stop commands when output is blocked by mode, timeout, or safety.
- Record runtime state through a standard rosbag profile.
- Replay runtime bags without connecting to real hardware adapters.

Implementation focus:

- strengthen mode transition tests
- expand command arbiter tests beyond import tests
- add safety manager unit tests for heartbeat and command timeout behavior
- add launch tests for teleop candidate command approval
- add launch tests for autonomy candidate command approval
- add launch tests for blocked-source behavior
- keep mock locomotion simple and clearly non-physical

Exit criteria:

- A developer can run the mock runtime and prove command flow from source to
  `cmd_vel/approved`.
- A runtime bag captures enough state to explain mode, safety, and command
  decisions.
- All active command paths have tests for allowed and blocked behavior.

## Phase 2: Teleoperation and Takeover

Goal: make human operation and takeover a first-class runtime path.

Tasks:

- Define operator heartbeat behavior and failure policy.
- Keep teleop candidate commands separate from approved commands.
- Add mode takeover tests from autonomy and AI policy to teleop.
- Add explicit rejection tests for takeover from inactive, maintenance, and
  shutdown.
- Add docs for keyboard teleop, joystick teleop, and remote operator profiles.
- Add diagnostics for teleop heartbeat freshness and takeover events.
- Add rosbag topics for teleop heartbeat and mode transition state.

Important behavior:

- Teleop should have higher priority than autonomy and AI policy.
- Takeover should be visible on `mode/transition_state`.
- Missing operator heartbeat should either warn or trigger MRM based on
  parameterized policy.
- The runtime must never assume cloud connectivity for emergency stop or local
  stabilization.

Exit criteria:

- An operator can take over from autonomy or AI policy in the mock runtime.
- Takeover is recorded, diagnosable, and replayable.
- Teleop loss behavior is documented and covered by tests.

## Phase 3: Safety and MRM

Goal: make minimal risk behavior predictable and inspectable.

Tasks:

- Expand watchdog coverage for command timeout, heartbeat timeout, hardware
  heartbeat timeout, battery, thermal, CPU, and stale runtime topics.
- Define safety state transition expectations for OK, WARN, FAULT, ESTOP, and
  MRM.
- Add tests for safety manager service-triggered MRM.
- Add tests for clearing MRM and rejecting clear when E-stop is active.
- Add incident logging conventions.
- Add safety docs showing how each command path can be blocked.

MRM behavior should include:

- stop approved output
- request operator intervention
- record reason and timestamp
- keep runtime state visible
- avoid hidden resets
- avoid automatic return to autonomy without explicit approval

Exit criteria:

- Each safety state has a documented effect on mode transitions and command
  output.
- A developer can force MRM, clear MRM, and replay the incident.
- Safety rules are covered by unit or launch tests.

## Phase 4: Locomotion and Nav2 Integration

Goal: connect standard navigation-style commands to humanoid locomotion
constraints without pretending to solve locomotion research.

Tasks:

- Keep `humaware_locomotion_interface` as a contract, not a controller.
- Treat vendor gait and low-level controllers as adapters.
- Keep Nav2 output as candidate autonomy command input.
- Add command limit diagnostics for linear and angular velocity clamps.
- Add capability states for stand, stop, walk velocity, turn in place, and
  recover posture.
- Add docs for how Nav2 goal movement maps to humanoid-safe velocity commands.
- Add blocked behavior for unsafe posture, fall risk, and unavailable locomotion.

Non-goals:

- new walking controller
- RL gait training
- full whole-body control
- physics-accurate simulator

Exit criteria:

- Nav2-style commands can flow through autonomy candidate topics and into the
  command arbiter.
- The runtime can explain why locomotion is unavailable, degraded, executing, or
  faulted.
- Command limits and source decisions are visible in diagnostics and logs.

## Phase 5: Hardware Adapter Template

Goal: make the first real robot adapter straightforward without baking one
vendor into the core.

Tasks:

- Add a generic hardware adapter template package.
- Define adapter responsibilities and forbidden behavior.
- Require hardware heartbeat publication.
- Require adapter identity metadata: robot model, firmware, SDK version, git SHA,
  and launch profile.
- Keep direct actuator control out of generic tests.
- Add adapter checklist docs.
- Add verified matrix format for robot support claims.

Adapter responsibilities:

- translate approved runtime commands to vendor commands
- publish hardware heartbeat
- publish or relay robot state
- expose adapter diagnostics
- stop output on runtime shutdown, MRM, E-stop, stale commands, or missing
  required state

Adapter forbidden behavior:

- bypass `humaware_safety_manager`
- subscribe directly to unapproved candidate command topics for hardware output
- publish direct hardware commands in tests
- claim real-robot support without logs and version metadata

Exit criteria:

- A new hardware adapter can be scaffolded from docs and template code.
- Adapter rules are testable.
- Real-robot support claims have a consistent evidence format.

## Phase 6: Rosbag-Native Operations

Goal: make logs and replay central to development and operations.

Tasks:

- Keep a minimal runtime recording profile.
- Add incident recording profile.
- Add replay-only launch profile that cannot connect to hardware adapters.
- Add bag evaluation tools for runtime topic freshness and state transitions.
- Add docs for debugging mode transitions, command arbitration, safety state,
  TF, DDS, and QoS from a bag.
- Add CI fixture bags only if they remain small and stable.

Runtime bags should answer:

- what mode was active
- who requested a mode transition
- what safety state was active
- what command source was selected
- why output was blocked
- when MRM triggered
- whether diagnostics were stale
- what capability was requested
- whether a skill command was published

Exit criteria:

- A developer can reproduce a runtime event from rosbag without a robot.
- Bags can be used to create actionable bug reports.
- Replay docs explicitly warn against replaying into real hardware command
  adapters.

## Phase 7: Diagnostics and Monitoring

Goal: make the runtime observable enough for repeated operation.

Tasks:

- Expand `HealthState` as needed without duplicating standard diagnostics.
- Keep `/diagnostics` integration standard.
- Add stale-topic detection tests.
- Add health fields for command output, nav output, safety state, mode, and MRM.
- Add operator-facing docs for interpreting health state.
- Add network and DDS troubleshooting profiles.
- Add system resource monitors when the dependency choice is clear.

Monitoring should cover:

- runtime topic freshness
- command output enabled or blocked
- safety state and MRM state
- active mode and requested mode
- capability availability
- teleop heartbeat freshness
- hardware heartbeat freshness
- CPU, GPU, memory, battery, and thermal state where available
- rosbag recording status

Exit criteria:

- Runtime health changes when key topics go stale.
- Diagnostics expose enough context for an operator console or Foxglove layout.
- Health and diagnostics are captured by the standard runtime bag profile.

## Phase 8: Distributed Execution

Goal: support realistic deployments across robot, operator station, edge server,
and cloud without moving safety-critical paths off robot.

Deployment roles:

- robot onboard computer: motor interface, local state, safety, low-latency
  perception, command arbitration
- operator station: teleop, RViz, Foxglove, debugging, takeover
- edge server: heavy perception, mapping, logging, policy inference when safe
- cloud: dataset storage, model registry, fleet dashboard, non-critical
  monitoring

Rules:

- E-stop, MRM, command timeout, and local stop behavior stay local.
- Cloud services must not be required for safe stop.
- ROS 2 and DDS are the default local graph.
- Zenoh bridge profiles can be added for remote or constrained links.
- Namespace conventions must work for one robot before scaling to many.

Exit criteria:

- Docs explain which runtime parts may be remote and which must stay local.
- Namespaces and topic contracts are compatible with multi-robot deployments.
- Network loss behavior is documented and testable.

## Phase 9: AI Policy Runtime Boundary

Goal: support AI policy integration without becoming an AI model project.

Tasks:

- Keep policy output as candidate actions.
- Add policy provider interface docs.
- Add policy safety gate docs and initial stubs only when runtime contracts are
  ready.
- Add examples for simple scripted policy providers before VLA integration.
- Add operator approval hooks for risky actions.
- Add logging fields for policy source, confidence, timeout, and rejected reason.
- Keep LeRobot and OpenVLA bridges experimental until validated.

Policy output must pass:

- active mode check
- capability availability check
- safety state check
- command limits
- timeout
- optional operator approval
- command arbitration
- incident logging

Exit criteria:

- A simple policy provider can request a skill without bypassing safety.
- Rejected policy actions are visible and logged.
- Policy runtime code remains separate from model training code.

## Phase 10: Manipulation Boundary

Goal: define safe integration points for manipulation without taking on full
whole-body manipulation too early.

Tasks:

- Add manipulation capability placeholders.
- Add MoveIt bridge design docs before implementation.
- Define arm safe pose behavior.
- Define locomotion and manipulation arbitration rules.
- Define what happens to arms during walking, stopping, and MRM.
- Add capability states for arm movement and grasp only when message contracts
  are clear.

Initial non-goals:

- dexterous manipulation
- bimanual task autonomy
- full whole-body MPC
- universal grasping stack

Exit criteria:

- The runtime has a clear place to integrate MoveIt.
- Walking and arm command conflicts have documented behavior.
- Arm safe pose is part of safety policy design.

## Phase 11: Fleet and Operations

Goal: prepare HumaWare for multiple robots without prematurely building a cloud
platform.

Tasks:

- Define namespace convention: `/hw/<robot_id>/...` or equivalent launch-time
  mapping.
- Add multi-robot launch examples with mock robots.
- Add fleet adapter boundary docs.
- Keep Open-RMF integration as an adapter, not a replacement.
- Add per-robot diagnostics and bag profiles.
- Add operator identity and request tracking fields where needed.

Fleet-level concerns:

- robot identity
- mode state per robot
- safety state per robot
- task ownership
- operator takeover
- remote diagnostics
- incident logs
- facility integration through Open-RMF or similar tools

Exit criteria:

- Two mock robots can run with isolated namespaces.
- Fleet-facing state can be read without knowing internal package details.
- Real fleet scheduling remains outside the stable core until needed.

## Phase 12: Release and Governance

Goal: make the project credible as reusable infrastructure.

Tasks:

- Add CODEOWNERS when package ownership becomes meaningful.
- Maintain release notes with public API changes.
- Keep monthly or milestone-based release cadence once real usage begins.
- Create working group labels only when contributors exist.
- Require experiment cards for real-robot PRs.
- Keep stable core smaller than experimental universe packages.

Release artifacts should include:

- version
- supported ROS distro
- verified robot matrix
- known safety limitations
- migration notes
- launch commands
- logs or bags for real-robot claims
- CI status

Exit criteria:

- Users can decide whether a release is suitable for simulation-only, lab robot,
  or real deployment work.
- Public message changes are rare and documented.
- Real-robot claims are evidence-based.

## Current Near-Term Backlog

The immediate sequence should be:

1. Add more command arbiter unit tests for source priority, timeout, mode
   mismatch, and safety blocking.
2. Add safety manager unit tests for heartbeat timeout, approved command timeout,
   MRM trigger, and MRM clear behavior.
3. Add launch test for teleop candidate command approval.
4. Add launch test for autonomy candidate command approval.
5. Add replay-only launch profile documentation and test that it does not start
   hardware adapters.
6. Add hardware adapter template package with strict README rules.
7. Add runtime bag evaluation tool scaffold.
8. Add diagnostics stale-topic tests.
9. Add multi-robot namespace design doc.
10. Add policy provider stub that only exercises candidate-action boundaries.

## Definition of Done

A HumaWare change is done when:

- it keeps the deployment-first direction intact
- it does not bypass safety manager or command arbiter
- public interfaces are documented
- tests cover the runtime rule being changed
- launch behavior remains reproducible
- logs or diagnostics can explain failures
- docs describe real-robot assumptions and limitations
- CI passes

For real-robot claims, done also requires:

- robot model
- firmware version
- vendor SDK version
- ROS distro
- environment description
- launch command
- git SHA
- expected behavior
- observed behavior
- logs and bag file where available

## Success Metrics

The project should measure operational quality before model quality:

- bringup success rate
- time to active mode
- command approval latency
- MRM trigger latency
- teleop takeover latency
- runtime topic freshness
- stale diagnostics detection latency
- command timeout behavior
- rosbag replay reproducibility
- adapter heartbeat reliability
- operator intervention frequency
- recovery success rate
- documented failure cases

Model accuracy, RL reward, or simulation realism may be useful for integrated
providers, but they are not primary HumaWare success metrics.

## Working Principle

Every new feature should make a real humanoid system easier to operate, inspect,
stop, recover, replay, or integrate. If a feature mainly improves a simulation
demo, model training loop, or one-off research result without strengthening the
runtime boundary, it belongs outside the HumaWare core.
