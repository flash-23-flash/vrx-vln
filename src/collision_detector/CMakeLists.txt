cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(collision_detector)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable
add_executable(collision_detector_node src/collision_detector_node.cpp)

# Link dependencies
ament_target_dependencies(collision_detector_node
  rclcpp
  std_msgs
  nav_msgs
)

# Install the executable
install(TARGETS
  collision_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
