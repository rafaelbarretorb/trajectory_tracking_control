// Copyright 2020
#include <ros/ros.h>
#include<trajectory_tracking_control/controller.h>
// #include<trajectory_tracking_control/controller_const_trajectory.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_control");
  ros::NodeHandle nh;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tf(buffer);
  trajectory_tracking_control::Controller controller("trajectory_tracking_control", &nh, buffer);
  // trajectory_tracking_control::ControllerConstTrajectory controller_const(&nh, buffer);

  ros::spin();
  return 0;
}
