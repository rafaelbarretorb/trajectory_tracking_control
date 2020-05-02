// Copyright 2020

#include <ros/ros.h>
#include <trajectory_tracking_control/controller.h>
#include <boost/bind.hpp>

namespace trajectory_tracking_control {

Controller::Controller(const std::string &name, ros::NodeHandle *nodehandle) :
                       action_name_(name),
                       nh_(*nodehandle),
                       as_(nh_, name, boost::bind(&Controller::executeCB, this, _1), false) {
  as_.start();
}

void Controller::executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {}

void Controller::requestReferenceMatrix() {
  // TODO(BARRETO) Request Service
}
}  // namespace trajectory_tracking_control
