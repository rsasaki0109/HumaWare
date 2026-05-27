# humaware_msgs

Stability: experimental.

This package defines the first humanoid-specific runtime state messages.

The package intentionally avoids low-level torque, joint, or whole-body control interfaces. Those should remain behind robot-specific adapters or standard ROS control interfaces until a stable public contract is justified.

Initial health summary:

- `HealthState`: aggregated runtime health for dashboards and smoke tests

Initial capability contracts:

- `Capability`: one callable robot capability and its safety constraints
- `CapabilityRegistry`: current robot capability set

Initial navigation bridge state:

- `NavigationBridgeState`: Nav2-style velocity bridge status and blocking reason
- `ModeTransitionState`: last mode transition request and outcome

Initial services:

- `ListCapabilities`: query all or selected runtime capabilities
- `SetMode`: request a mode transition through the mode manager
- `Takeover`: request operator takeover into teleop mode
- `TriggerMRM`: trigger a minimal risk maneuver through the safety manager
- `ClearMRM`: clear a manually triggered minimal risk maneuver
