# Package Naming

HumaWare packages use explicit ROS 2 package names prefixed with `humaware_`.

## Repository Groups

- `humaware_msgs`: internal runtime interfaces
- `humaware_adapi_msgs`: stable external API interfaces
- `humaware_core`: stable runtime utilities and contracts
- `humaware_runtime`: mode, task, behavior tree, and arbitration runtime
- `humaware_safety`: safety manager, watchdogs, MRM, limits, incident logging
- `humaware_locomotion`: locomotion interfaces and navigation bridge
- `humaware_navigation`: map, localization, and Nav2 profiles
- `humaware_manipulation`: MoveIt and arm safety integration
- `humaware_perception`: perception providers and object interfaces
- `humaware_teleop`: local and remote operator control
- `humaware_ai`: policy runtime and policy bridge integrations
- `humaware_hardware`: real and mock robot adapters
- `humaware_sim`: simulator adapters and scenarios
- `humaware_fleet`: Open-RMF and multi-robot conventions
- `humaware_launch`: launch profiles
- `humaware_examples`: runnable examples
- `humaware_tools`: CLI, bag evaluation, diagnostics, calibration

## Stability Labels

Package READMEs should state one of:

- `stable`: API compatibility is expected across minor releases
- `experimental`: API can change between minor releases
- `template`: intended as a starting point for downstream adapters
- `deprecated`: scheduled for removal with migration notes

## Robot Adapter Names

Robot-specific packages should be precise:

- use `humaware_unitree_h1` for verified Unitree H1 support
- use `humaware_unitree_g1_experimental` until G1 real-robot claims are verified
- avoid broad names such as `humaware_unitree_all`
