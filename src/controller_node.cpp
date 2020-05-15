// Copyright 2020
#include <ros/ros.h>
#include<trajectory_tracking_control/controller.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_control");

  ros::NodeHandle nh;
  trajectory_tracking_control::Controller controller("trajectory_tracking_control", &nh);
  ros::spin();
  return 0;
}
