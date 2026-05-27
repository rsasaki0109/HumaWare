# humaware_mock_locomotion_adapter

Stability: experimental.

Mock locomotion adapter for launch, testing, and tooling validation.

The adapter subscribes to `cmd_vel/approved`, applies a simple velocity threshold, and publishes `locomotion/state`. It does not simulate physics or robot dynamics.

## Behavior

- nonzero approved velocity: `WALKING` or `TURNING`
- zero approved velocity: `STOPPING`, then `STANDING`
- command timeout: `STOPPING`, then `STANDING`
- inactive or maintenance mode: `INACTIVE`
- fault, shutdown, E-stop, or MRM: `STOPPING`
