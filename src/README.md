# Source Layout

The repository starts with a minimal ROS 2 workspace and grows toward the full HumaWare structure.

Initial packages:

- `humaware_msgs`: humanoid runtime state and capability interfaces
- `humaware_mock_robot`: mock state publisher for launch and tooling validation
- `humaware_safety_manager`: first safety state publisher and MRM boundary
- `humaware_mode_manager`: authoritative runtime mode state and mode transition service
- `humaware_command_arbiter`: mode-aware and safety-aware candidate command gate
- `humaware_locomotion_interface`: approved command to locomotion-state adapter contract
- `humaware_mock_locomotion_adapter`: mock adapter for locomotion-state validation
- `humaware_keyboard_teleop`: keyboard operator candidate command source
- `humaware_nav2_bridge`: Nav2-style velocity command to autonomy candidate bridge
- `humaware_diagnostics_aggregator`: runtime health and diagnostics summary
- `humaware_bag_profiles`: runtime rosbag record and replay profiles
- `humaware_launch`: bringup launch profiles
- `humaware_examples`: example launch profiles

Planned package groups:

- `humaware_core`
- `humaware_runtime`
- `humaware_safety`
- `humaware_diagnostics`
- `humaware_locomotion`
- `humaware_navigation`
- `humaware_manipulation`
- `humaware_perception`
- `humaware_teleop`
- `humaware_ai`
- `humaware_hardware`
- `humaware_sim`
- `humaware_fleet`
- `humaware_tools`
