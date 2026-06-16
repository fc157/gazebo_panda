cmake_minimum_required(VERSION 3.16.3)
project(moveit_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Enable Qt AUTOMOC (process Q_OBJECT macros in headers)
set(CMAKE_AUTOMOC ON)

# Build the C++ MoveIt control GUI executable
add_executable(moveit_control_gui
  src/moveit_control_gui.cpp
  include/moveit_control_gui/main_window.hpp
)
target_include_directories(moveit_control_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(moveit_control_gui PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(moveit_control_gui
  "rclcpp"
  "rclcpp_action"
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "geometry_msgs"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "tf2"
)

target_link_libraries(moveit_control_gui
  Qt5::Widgets
)

install(TARGETS moveit_control_gui
  DESTINATION lib/${PROJECT_NAME})

# Install config and launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()