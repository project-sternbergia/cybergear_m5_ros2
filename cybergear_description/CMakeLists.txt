cmake_minimum_required(VERSION 3.8)
project(cybergear_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  meshes
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright checks
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
endif()

ament_package()
