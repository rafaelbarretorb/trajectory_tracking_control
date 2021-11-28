/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_  // NOLINT
#define TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <eigen3/Eigen/Core>   // MatrixXd

// Service
#include "trajectory_tracking_control/ComputeReferenceStates.h"

using Eigen::MatrixXd;

namespace trajectory_tracking_control {

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(ros::NodeHandle *nodehandle);

  void makeConstantTrajectory(double t_sampling, MatrixXd &ref_states_matrix);

  void makeTrajectory(const geometry_msgs::PoseArray &path,
                      double vel_avg,
                      double t_sampling,
                      MatrixXd &ref_states_matrix);

  double getGoalDistance() const;
  
 private:
  void displayConstantTrajectoryInfo(double x_offset, double y_offset, double x_amp, double y_amp, double freq);
  ros::ServiceClient ref_states_srv_;
  double goal_distance_;
  ros::NodeHandle nh_;
};

}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_
