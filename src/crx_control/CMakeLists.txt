cmake_minimum_required(VERSION 3.8)
project(crx_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gazebo_description_parameters test/test_gazebo_description.py)
  ament_add_pytest_test(camera_geometry test/test_camera_geometry.py)
endif()

ament_package()
