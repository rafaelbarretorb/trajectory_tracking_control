// Copyright 2020
#include <ros/ros.h>

// #include<trajectory_tracking_control/controller_const_trajectory.h>
#include <tf2_ros/transform_listener.h>
#include <string>

#include "trajectory_tracking_control/controller.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_controller_node");
  ros::NodeHandle nh;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tf(buffer);

  std::string controller_type;
  ros::NodeHandle("~").getParam("controller_type", controller_type);

  if (controller_type.compare("Linear") == 0) {
    ROS_INFO("Controller type: %s", controller_type.c_str());
    trajectory_tracking_control::Controller controller(controller_type, "trajectory_tracking_control", &nh, buffer);
  }

  if (controller_type.compare("MPC") == 0) {}

  ros::spin();
  return 0;
}
