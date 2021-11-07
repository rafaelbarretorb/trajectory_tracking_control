cmake_minimum_required(VERSION 3.1.3)
project(trajectory_tracking_control)

# TODO chech install action messages to avoid repeat catkin_make

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

add_service_files(DIRECTORY srv
  FILES ComputeReferenceStates.srv
)

add_action_files(DIRECTORY action
	FILES ExecuteTrajectoryTracking.action
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
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
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}

    # Set manually because Eigen sets a non standard INCLUDE DIR
    ${Eigen3_INCLUDE_DIRS}
)

add_executable(
  trajectory_tracking_control
  src/controller_node.cpp
#   src/controller_const_trajectory.cpp
  src/pose_handler.cpp
  src/controller.cpp
#   src/controller_test_node.cpp
#   src/test_localization.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_tracking_control  ${catkin_LIBRARIES})
