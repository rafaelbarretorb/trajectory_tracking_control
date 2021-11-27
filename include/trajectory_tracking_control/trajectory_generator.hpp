/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_  // NOLINT
#define TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_


#include <geometry_msgs/PoseArray.h>

#include <eigen3/Eigen/Core>   // MatrixXd

// Service
#include "trajectory_tracking_control/ComputeReferenceStates.h"

using Eigen::MatrixXd;

namespace trajectory_tracking_control {

class TrajectoryGenerator {
 public:
  TrajectoryGenerator() = default;

  void makeConstantTrajectory(double vel_avg,  // TODO(RAFael) not using vel_avg
                              double t_sampling,
                              MatrixXd &ref_states_matrix);

  void makeTrajectory(const geometry_msgs::PoseArray &path,
                      double vel_avg,
                      double t_sampling,
                      MatrixXd &ref_states_matrix);

 private:
  bool const_trajectory_{false};
  ros::ServiceClient ref_states_srv_;
  double goal_distance_;  // TODO(Rafael) DO I need this?
};

}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_
