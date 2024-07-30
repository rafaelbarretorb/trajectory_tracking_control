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
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

add_library(
  trajectory_controller_lib
  src/controller_node.cpp
  src/trajectory_generator.cpp
)

target_compile_definitions(trajectory_controller_lib
  PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(trajectory_controller_lib rclcpp std_msgs)

add_executable(control_node src/controller_node.cpp)

target_link_libraries(
  control_node
  trajectory_controller_lib
)

ament_target_dependencies(control_node rclcpp)

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
