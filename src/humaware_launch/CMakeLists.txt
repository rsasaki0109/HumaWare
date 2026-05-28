cmake_minimum_required(VERSION 3.8)
project(humaware_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_mock_bringup_launch.py
    TARGET test_mock_bringup_launch
    TIMEOUT 45
  )
  set_tests_properties(
    test_mock_bringup_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=91"
  )

  add_launch_test(
    test/test_candidate_command_paths.py
    TARGET test_candidate_command_paths
    TIMEOUT 45
  )
  set_tests_properties(
    test_candidate_command_paths
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=92"
  )

  add_launch_test(
    test/test_replay_only_launch.py
    TARGET test_replay_only_launch
    TIMEOUT 45
  )
  set_tests_properties(
    test_replay_only_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=93"
  )

  add_launch_test(
    test/test_scripted_policy_launch.py
    TARGET test_scripted_policy_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_scripted_policy_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=94"
  )

  add_launch_test(
    test/test_takeover_launch.py
    TARGET test_takeover_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_takeover_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=95"
  )

  add_launch_test(
    test/test_arbiter_priority_launch.py
    TARGET test_arbiter_priority_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_arbiter_priority_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=96"
  )

  add_launch_test(
    test/test_mrm_blocks_output_launch.py
    TARGET test_mrm_blocks_output_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_mrm_blocks_output_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=97"
  )

  add_launch_test(
    test/test_hardware_adapter_template_launch.py
    TARGET test_hardware_adapter_template_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_hardware_adapter_template_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=98"
  )

  add_launch_test(
    test/test_heartbeat_timeout_mrm_launch.py
    TARGET test_heartbeat_timeout_mrm_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_heartbeat_timeout_mrm_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=99"
  )

  add_launch_test(
    test/test_hardware_heartbeat_timeout_mrm_launch.py
    TARGET test_hardware_heartbeat_timeout_mrm_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_hardware_heartbeat_timeout_mrm_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=100"
  )

  add_launch_test(
    test/test_nav2_bridge_autonomy_launch.py
    TARGET test_nav2_bridge_autonomy_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_nav2_bridge_autonomy_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=101"
  )

  add_launch_test(
    test/test_nav2_bridge_mrm_blocks_launch.py
    TARGET test_nav2_bridge_mrm_blocks_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_nav2_bridge_mrm_blocks_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=102"
  )

  add_launch_test(
    test/test_approved_command_timeout_mrm_launch.py
    TARGET test_approved_command_timeout_mrm_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_approved_command_timeout_mrm_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=103"
  )

  add_launch_test(
    test/test_estop_blocks_output_launch.py
    TARGET test_estop_blocks_output_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_estop_blocks_output_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=104"
  )

  add_launch_test(
    test/test_watchdog_mrm_auto_recovery_launch.py
    TARGET test_watchdog_mrm_auto_recovery_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_watchdog_mrm_auto_recovery_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=105"
  )

  add_launch_test(
    test/test_safety_gates_mode_escalation_launch.py
    TARGET test_safety_gates_mode_escalation_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_safety_gates_mode_escalation_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=106"
  )

  add_launch_test(
    test/test_teleop_drive_then_stale_stop_launch.py
    TARGET test_teleop_drive_then_stale_stop_launch
    TIMEOUT 60
  )
  set_tests_properties(
    test_teleop_drive_then_stale_stop_launch
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=107"
  )
endif()

ament_package()
