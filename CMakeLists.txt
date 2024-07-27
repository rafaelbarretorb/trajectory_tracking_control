cmake_minimum_required(VERSION 3.5)
project(trajectory_tracking_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(control_node src/controller_node.cpp)
ament_target_dependencies(control_node rclcpp std_msgs)

install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
