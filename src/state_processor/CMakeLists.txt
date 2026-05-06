cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(state_processor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cluster_msg REQUIRED)
find_package(state_msg REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable
add_executable(state_processor_node src/state_processor_node.cpp)

# Link dependencies
ament_target_dependencies(state_processor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cluster_msg
  state_msg
  message_filters
  std_msgs
  nav_msgs
)

# Install the executable
install(TARGETS
  state_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
