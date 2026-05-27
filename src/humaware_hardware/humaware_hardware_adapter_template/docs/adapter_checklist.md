# Hardware adapter pre-deployment checklist

This checklist must be completed before merging a HumaWare hardware
adapter into the stable core and before claiming real-robot support.

## Contract

- [ ] Package name follows the `humaware_<vendor>_<robot>_adapter` form.
- [ ] Adapter subscribes only to `cmd_vel/approved` for actuation
      (never to candidate topics such as `teleop/cmd_vel`).
- [ ] Adapter publishes `hardware/heartbeat` at a documented rate.
- [ ] Adapter publishes vendor robot state under the runtime namespace.
- [ ] Adapter exposes diagnostics that include adapter identity.
- [ ] Adapter stops output on shutdown, MRM, E-stop, stale commands,
      mode change, or missing required state.
- [ ] Adapter is excluded from the `replay_only.launch.py` profile.
- [ ] Adapter has no direct dependency on Nav2, MoveIt, Open-RMF, or any
      foundation-model SDK.

## Identity metadata

Every adapter must publish the following identity values through ROS
parameters and diagnostics. Missing fields disqualify the adapter from
real-robot support claims.

- [ ] `robot_model`
- [ ] `firmware_version`
- [ ] `sdk_version`
- [ ] `git_sha`
- [ ] `launch_profile`

## Evidence

- [ ] Bringup log captured in `logs/` or referenced from release notes.
- [ ] Runtime rosbag captured for at least one supported mode.
- [ ] MRM trigger demonstrated and recorded.
- [ ] E-stop behavior verified with the vendor hardware.
- [ ] Adapter heartbeat freshness verified with the safety manager.
- [ ] Diagnostics inspected on `/diagnostics` and in a Foxglove layout.

## Tests

- [ ] Adapter unit tests cover the gate logic for OK, WARN, FAULT,
      ESTOP, MRM, and stale command paths.
- [ ] No test publishes directly to actuator topics. Generic CI tests
      must remain hardware-free.
- [ ] Integration tests verify that the adapter does not actuate when
      runtime mode is inactive, fault, maintenance, or shutdown.

## Documentation

- [ ] README.md states stability level, supported robot, supported ROS
      distro, and known limitations.
- [ ] Verified matrix entry (see `verified_matrix_template.md`)
      provided with reproducible launch command and git SHA.
- [ ] Migration notes added when the adapter changes a public runtime
      message or service contract.
