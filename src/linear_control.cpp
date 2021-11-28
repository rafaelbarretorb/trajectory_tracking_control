/*
  Copyright 2021 - Rafael Barreto
*/

#include "trajectory_tracking_control/linear_control.hpp"

namespace trajectory_tracking_control {

LinearControl::LinearControl(tf2_ros::Buffer& tf_buffer,
                             const MatrixXd &ref_states_matrix) : pose_handler_(&tf_buffer){}  

bool LinearControl::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  double omega;
  double vel;

  // Update robot pose
  curr_pose_ = pose_handler_.getRobotPose();
  yaw_curr_ = pose_handler_.getYawFromQuaternion(curr_pose_.orientation);
  q_curr_ << curr_pose_.position.x, curr_pose_.position.y, yaw_curr_;


  // Transform to global coordinate
  tf_to_global_ << cos(yaw_curr_), sin(yaw_curr_), 0.0,
                   -sin(yaw_curr_), cos(yaw_curr_), 0.0,
                   0.0, 0.0, 1.0;

  // Posture Error
  error_ = tf_to_global_ * (q_ref_ - q_curr_);

  // Wrap to PI yaw error
  error_(2, 0) = angles::normalize_angle(error_(2, 0));

  // Control
  error_x_ = error_(0, 0);
  error_y_ = error_(1, 0);
  error_yaw_ = error_(2, 0);

  if (!constant_gains_) {
    // Variable gains
    k_x_ = 2*zeta_*sqrt(omega_ref_*omega_ref_ + g_*vel_ref_*vel_ref_);
    k_y_ = g_*vel_ref_;
    k_yaw_ = k_x_;
  }

  // Compute Velocities
  vel = vel_ref_*cos(error_yaw_) + k_x_*error_x_;
  omega = omega_ref_ + k_y_*error_y_ + k_yaw_*error_yaw_;

  // Ensure that the angular velocity does not exceed the maximum allowed
  if (fabs(omega) > omega_max_) {
    omega = copysign(omega_max_, omega);
  }

  if (vel < 0.0) {
    vel = 0.0;
  }

  // Ensure that the linear velocity does not exceed the maximum allowed
  if (vel > vel_max_) {
    vel = vel_max_;
  }

  cmd_vel.linear.x = vel;
  cmd_vel.angular.z = omega;

  return true;
}
}  