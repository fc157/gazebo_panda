cmake_minimum_required(VERSION 3.16.3)
project(panda_moveit_config)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build the C++ MoveIt test executable
add_executable(moveit_test src/moveit_test.cpp)
target_include_directories(moveit_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(moveit_test
  "rclcpp"
  "rclcpp_action"
  "control_msgs"
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "geometry_msgs"
)

install(TARGETS moveit_test
  DESTINATION lib/${PROJECT_NAME})

# Install config and launch files
install(DIRECTORY
  config
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
