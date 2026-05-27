cmake_minimum_required(VERSION 3.8)
project(humaware_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/BalanceState.msg"
  "msg/Capability.msg"
  "msg/CapabilityRegistry.msg"
  "msg/CommandArbitrationState.msg"
  "msg/FootContactState.msg"
  "msg/HealthState.msg"
  "msg/HumanoidState.msg"
  "msg/LocomotionState.msg"
  "msg/ModeState.msg"
  "msg/ModeTransitionState.msg"
  "msg/MRMState.msg"
  "msg/NavigationBridgeState.msg"
  "msg/SafetyState.msg"
)

set(srv_files
  "srv/ClearMRM.srv"
  "srv/ListCapabilities.srv"
  "srv/SetMode.srv"
  "srv/Takeover.srv"
  "srv/TriggerMRM.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
