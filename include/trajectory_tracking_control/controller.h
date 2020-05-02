
// Copyright Rafael Barreto

#ifndef TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_ // NOLINT
#define TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_

#include <ros/ros.h>

// Messages
#include <geometry_msgs/Point.h>


// Action
#include <actionlib/server/simple_action_server.h>
#include <trajectory_tracking_control/ExecuteTrajectoryTrackingAction.h>

#include <vector>
#include <string>

#include <boost/bind.hpp>

namespace trajectory_tracking_control {

typedef actionlib::SimpleActionServer<trajectory_tracking_control::ExecuteTrajectoryTrackingAction>
  ExecuteTrajectoryTrackingActionServer;


class Controller {
 public:
  Controller(const std::string &name, ros::NodeHandle *nodehandle);

  void executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal);

  void requestReferenceMatrix();

 protected:
  std::string action_name_;
  ros::NodeHandle nh_;
  ExecuteTrajectoryTrackingActionServer as_;
  std::vector<geometry_msgs::Point> reference_trajectory_;

};
};  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_ NOLINT
