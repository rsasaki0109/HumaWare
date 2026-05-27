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
endif()

ament_package()
