
cmake_minimum_required(VERSION 3.10)
project(planner)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# A* pathfinding library
add_library(planner_lib
  src/planner_core.cpp
)

target_include_directories(planner_lib
  PUBLIC include
)

ament_target_dependencies(planner_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

# Planner ROS2 node
add_executable(planner_node
  src/planner_node.cpp
)

ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)

target_link_libraries(planner_node
  planner_lib
)

install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()