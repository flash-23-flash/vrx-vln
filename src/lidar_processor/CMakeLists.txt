cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(lidar_processor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cluster_msg REQUIRED)

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable
add_executable(lidar_processor_node src/lidar_processor_node.cpp)

# # Include directories
# target_include_directories(lidar_processor_node
#   PRIVATE
#   include
# )

# Link dependencies
ament_target_dependencies(lidar_processor_node
  rclcpp
  sensor_msgs
  cluster_msg
)

# Install the executable
install(TARGETS
  lidar_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
