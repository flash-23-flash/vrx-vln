cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(action_planner)

# Add libtorch (modify to your own location)
list(APPEND CMAKE_PREFIX_PATH "/opt/libtorch")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(state_msg REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_info_msg REQUIRED)
find_package(Torch REQUIRED)

# Include directories (modify to your own location)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${TORCH_INCLUDE_DIRS}
)

# Add executable
add_executable(action_planner_node src/action_planner_node.cpp)

# Link dependencies
ament_target_dependencies(action_planner_node
  rclcpp
  state_msg
  std_msgs
  robot_info_msg
)

target_link_libraries(action_planner_node ${TORCH_LIBRARIES})

# Install the executable
install(TARGETS
  action_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
