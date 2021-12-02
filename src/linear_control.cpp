/*
  Copyright 2021 - Rafael Barreto
*/

#include "trajectory_tracking_control/linear_control.hpp"

namespace trajectory_tracking_control {

LinearControl::LinearControl(TrajectoryGenerator *trajectory_generator,
                             PoseHandler *pose_handler) : pose_handler_(*pose_handler),
                                                           traj_gen_(*trajectory_generator), {
  // Initialize matrices
  q_ref_ = MatrixXd(3, 1);
  q_curr_ = MatrixXd(3, 1);
  tf_to_global_ = MatrixXd(3, 3);
  error_ = MatrixXd(3, 1);
}

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

  // Pose Error
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

void LinearControl::updateReferenceState(int n) {
  x_ref_ = ref_states_matrix_(0, n);
  y_ref_ = ref_states_matrix_(1, n);
  dx_ref_ = ref_states_matrix_(2, n);
  dy_ref_ = ref_states_matrix_(3, n);
  ddx_ref_ = ref_states_matrix_(4, n);
  ddy_ref_ = ref_states_matrix_(5, n);

  yaw_ref_ = atan2(dy_ref_, dx_ref_);

  // Reference Pose State
  q_ref_ << x_ref_, y_ref_, yaw_ref_;

  // Reference Velocities
  vel_ref_ = sqrt(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);
  omega_ref_ = (dx_ref_*ddy_ref_ - dy_ref_*ddx_ref_)/(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);
}

void LinearControl::loadControllerParams() {
  ros::NodeHandle private_nh("~");

  std::string controller_type = "Linear";
  private_nh.getParam(controller_type + "/" + "constant_gains", constant_gains_);
  private_nh.getParam(controller_type + "/" + "k_x", k_x_);
  private_nh.getParam(controller_type + "/" + "k_y", k_y_);
  private_nh.getParam(controller_type + "/" + "k_yaw", k_yaw_);
  private_nh.getParam(controller_type + "/" + "g", g_);
  private_nh.getParam(controller_type + "/" + "zeta", zeta_);

  if (g_ < 0.0) {
    ROS_ERROR("Invalid design parameter.");
    throw std::invalid_argument("The designer parameter g received a negative value.");
  }
}

void LinearControl::displayControllerInfo() {}

void LinearControl::updateReferenceState(double time) {
  traj_gen_.updateReferenceState(time);
}

bool LinearControl::isGoalReached() {

}


void LinearControl::fillReferencePath(std::pair<double, double> *path) {
  traj_gen_.fillReferencePath(path)
}


}  // namespace trajectory_tracking_control
