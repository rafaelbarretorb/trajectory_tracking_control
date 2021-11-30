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
  explicit TrajectoryGenerator(ros::NodeHandle *nodehandle, double sampling_time);

  void makeConstantTrajectory();

  void makeTrajectory(const geometry_msgs::PoseArray &path, double vel_avg);

  double getGoalDistance() const;

  void updateReferenceState(double time);

 private:
  void displayConstantTrajectoryInfo(double x_offset, double y_offset, double x_amp, double y_amp, double freq);
  ros::ServiceClient ref_states_srv_;
  double goal_distance_;
  ros::NodeHandle nh_;
  MatrixXd ref_states_matrix_;
  double sampling_time_;

  // Current reference velocities
  double vel_ref_, omega_ref_;

  // Current reference state
  double x_ref_, y_ref_, dx_ref_, dy_ref_, ddx_ref_, ddy_ref_;

  // Yaw angle
  double yaw_ref_, yaw_curr_;

  // Reference Pose State (x, y, yaw)
  MatrixXd q_ref_;
};

}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_
