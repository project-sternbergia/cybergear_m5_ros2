cmake_minimum_required(VERSION 3.8)
project(cybergear_m5_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Get async communication library for serial connection
include(FetchContent)
FetchContent_Declare(async_comm
  GIT_REPOSITORY https://github.com/dpkoch/async_comm.git
  GIT_TAG master)
list(APPEND CMAKE_MODULE_PATH "${async_comm_SOURCE_DIR}")
FetchContent_MakeAvailable(async_comm)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(async_comm)

# create cybergear_m5_driver
add_library(cybergear_m5_driver
  src/cybergear_m5_driver.cpp
  src/cybergear_m5_packet.cpp
)
ament_target_dependencies(cybergear_m5_driver
  rclcpp
)
target_link_libraries(cybergear_m5_driver
  async_comm
)
include_directories(cybergear_m5_driver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${async_comm_INCLUDE_DIRS}/../
  ${async_comm_SOURCE_DIR}/include
)

# create executable
add_executable(cybergear_m5_bridge_node
  src/cybergear_m5_bridge_node.cpp
)
include_directories(cybergear_m5_bridge_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${async_comm_INCLUDE_DIRS}/../
  ${async_comm_SOURCE_DIR}/include
)
ament_target_dependencies(cybergear_m5_bridge_node
  rclcpp
  sensor_msgs
)
target_link_libraries(cybergear_m5_bridge_node
  async_comm
  cybergear_m5_driver
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  cybergear_m5_bridge_node
  cybergear_m5_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
