
// Copyright Rafael Barreto

#ifndef TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_ // NOLINT
#define TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_

#include <ros/ros.h>

// Messages
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

// Eigen
#include <Eigen/Core>

// Action
#include <actionlib/server/simple_action_server.h>
#include <trajectory_tracking_control/ExecuteTrajectoryTrackingAction.h>

// Service
#include <trajectory_tracking_control/ComputeReferenceStates.h>

#include <vector>
#include <string>

#include <boost/bind.hpp>

using Eigen::MatrixXd;

namespace trajectory_tracking_control {

typedef actionlib::SimpleActionServer<trajectory_tracking_control::ExecuteTrajectoryTrackingAction>
  ExecuteTrajectoryTrackingActionServer;


class Controller {
 public:
  Controller(const std::string &name, ros::NodeHandle *nodehandle);

  void executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal);

  void requestReferenceMatrix(const geometry_msgs::PoseArray &path, double vel_avg, double t_sampling);

 protected:
  std::string action_name_;
  ros::NodeHandle nh_;
  ExecuteTrajectoryTrackingActionServer as_;
  std::vector<geometry_msgs::Point> reference_trajectory_;

  // Matrix
  MatrixXd m_ref_states_;

  // 
  ros::ServiceClient ref_states_srv_;
};
};  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_ NOLINT
