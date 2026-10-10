cmake_minimum_required(VERSION 3.8)
project(crx_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(launch_contracts test/test_launch_contracts.py)
  ament_add_pytest_test(pick_place_integration test/test_pick_place_integration.py TIMEOUT 420)
endif()

ament_package()
