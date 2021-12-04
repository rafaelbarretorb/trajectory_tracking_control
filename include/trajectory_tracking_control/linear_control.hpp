/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_LINEAR_CONTROL_HPP_
#define TRAJECTORY_TRACKING_CONTROL_LINEAR_CONTROL_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Angles
#include <angles/angles.h>

#include <trajectory_tracking_control/ExecuteTrajectoryTrackingAction.h>

// Eigen
#include <eigen3/Eigen/Core>

#include <vector>
#include <utility>

#include "trajectory_tracking_control/controller.hpp"
// #include "trajectory_tracking_control/pose_handler.hpp"
// #include "trajectory_tracking_control/trajectory_generator.hpp"

using Eigen::MatrixXd;

namespace trajectory_tracking_control {

class LinearControl : public Controller {
 public:
  LinearControl(ros::NodeHandle *nodehandle,
                const ExecuteTrajectoryTrackingGoalConstPtr &goal);

  // bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);  // NOLINT

  // void updateReferenceState(int n);

  void loadControllerParams();

  void displayControllerInfo();

  // void updateReferenceState(double time);

  // bool isGoalReached();

  // double getReferenceX() const;

  // double getReferenceY() const;

  // double getReferenceYaw() const;

  // double getReferenceLinearVelocity() const;

  // double getReferenceAngularVelocity() const;

  // void fillReferencePath(std::vector<std::pair<double, double>> *path);

  void initializeMatrices();
 private:
  /*
    Reference States Matrix(6, n)

    | x_0    x_1    ...  x_n   |
    | y_0    y_1    ...  y_n   |
    | dx_0   dx_1   ...  dx_n  |
    | dy_0   dy_1   ...  dy_n  |
    | ddx_0  ddx_1  ...  ddx_n |
    | ddy_0  ddy_1  ...  ddy_n |

  */
  MatrixXd ref_states_matrix_;

  // Current Pose
  geometry_msgs::Pose curr_pose_;

  // Pose Error Matrix (3x1)
  MatrixXd error_;

  // Transform to global coordinate matrix (3x3)
  MatrixXd tf_to_global_;

  // Current Pose State (x, y, yaw)
  MatrixXd q_curr_;

  // Reference Pose State (x, y, yaw)
  MatrixXd q_ref_;

  // Pose Errors
  double error_x_, error_y_, error_yaw_;

  // Gains
  double k_x_, k_y_, k_yaw_;

  // Control design parameters
  double g_, zeta_;

  // Maximum absolute linear and angular velocities
  double vel_max_, omega_max_;

  // Current reference velocities
  double vel_ref_, omega_ref_;

  // Current reference state
  double x_ref_, y_ref_, dx_ref_, dy_ref_, ddx_ref_, ddy_ref_;

  // Yaw angle
  double yaw_ref_, yaw_curr_;

  // PoseHandler pose_handler_;

  bool const_gains_{false};

  bool const_trajectory_;

  double sampling_time_;

  // TrajectoryGenerator traj_gen_;
};

}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_LINEAR_CONTROL_HPP_
