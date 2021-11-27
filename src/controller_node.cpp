// Copyright 2020
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include "trajectory_tracking_control/controller.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_controller");
  ros::NodeHandle nh;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tf(buffer);

  trajectory_tracking_control::Controller controller("trajectory_tracking_control", &nh, buffer);

  ros::spin();
  return 0;
}
