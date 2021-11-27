cmake_minimum_required(VERSION 3.1.3)
project(trajectory_tracking_control)

# TODO check install action messages to avoid repeat catkin_make

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

add_service_files(DIRECTORY srv
  FILES
  ComputeReferenceStates.srv
)

add_action_files(DIRECTORY action
	FILES
  ExecuteTrajectoryTracking.action
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    actionlib_msgs
    actionlib
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(
  trajectory_controller_lib
  src/controller_node.cpp
  src/pose_handler.cpp
  src/controller.cpp
  src/trajectory_generator.cpp
)

add_dependencies(trajectory_controller_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(
  trajectory_controller
  src/controller_node.cpp
)

target_link_libraries(
  trajectory_controller
  trajectory_controller_lib
  ${catkin_LIBRARIES}
)
