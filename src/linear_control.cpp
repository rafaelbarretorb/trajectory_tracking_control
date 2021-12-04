/*
  Copyright 2021 - Rafael Barreto
*/

#include "trajectory_tracking_control/linear_control.hpp"

namespace trajectory_tracking_control {

LinearControl::LinearControl(ros::NodeHandle *nodehandle,
                             const ExecuteTrajectoryTrackingGoalConstPtr &goal) :
                             const_trajectory_(goal->const_trajectory),
                             sampling_time_(goal->sampling_time) {
  loadControllerParams();

  displayControllerInfo();

  initializeMatrices();
}

// bool LinearControl::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
//   double omega;
//   double vel;

//   // Update robot pose
//   curr_pose_ = pose_handler_.getRobotPose();
//   yaw_curr_ = pose_handler_.getYawFromQuaternion(curr_pose_.orientation);
//   q_curr_ << curr_pose_.position.x, curr_pose_.position.y, yaw_curr_;


//   // Transform to global coordinate
//   tf_to_global_ << cos(yaw_curr_), sin(yaw_curr_), 0.0,
//                    -sin(yaw_curr_), cos(yaw_curr_), 0.0,
//                    0.0, 0.0, 1.0;

//   // Pose Error
//   error_ = tf_to_global_ * (q_ref_ - q_curr_);

//   // Wrap to PI yaw error
//   error_(2, 0) = angles::normalize_angle(error_(2, 0));

//   // Control
//   error_x_ = error_(0, 0);
//   error_y_ = error_(1, 0);
//   error_yaw_ = error_(2, 0);

//   if (!const_gains_) {
//     // Variable gains
//     k_x_ = 2*zeta_*sqrt(omega_ref_*omega_ref_ + g_*vel_ref_*vel_ref_);
//     k_y_ = g_*vel_ref_;
//     k_yaw_ = k_x_;
//   }

//   // Compute Velocities
//   vel = vel_ref_*cos(error_yaw_) + k_x_*error_x_;
//   omega = omega_ref_ + k_y_*error_y_ + k_yaw_*error_yaw_;

//   // Ensure that the angular velocity does not exceed the maximum allowed
//   if (fabs(omega) > omega_max_) {
//     omega = copysign(omega_max_, omega);
//   }

//   if (vel < 0.0) {
//     vel = 0.0;
//   }

//   // Ensure that the linear velocity does not exceed the maximum allowed
//   if (vel > vel_max_) {
//     vel = vel_max_;
//   }

//   cmd_vel.linear.x = vel;
//   cmd_vel.angular.z = omega;

//   return true;
// }

// void TrajectoryControlROS::makeTrajectory(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
//   if (goal->const_trajectory) {
//     traj_gen.makeConstantTrajectory();
//   } else {
//     // Set goal position
//     goal_position_.x = goal->path.poses[goal->path.poses.size() - 1].position.x;
//     goal_position_.y = goal->path.poses[goal->path.poses.size() - 1].position.y;

//     traj_gen.makeTrajectory(goal->path, goal->average_velocity);
//     goal_distance_ = traj_gen.getGoalDistance();
//   }
// }

// void LinearControl::updateReferenceState(int n) {
//   x_ref_ = ref_states_matrix_(0, n);
//   y_ref_ = ref_states_matrix_(1, n);
//   dx_ref_ = ref_states_matrix_(2, n);
//   dy_ref_ = ref_states_matrix_(3, n);
//   ddx_ref_ = ref_states_matrix_(4, n);
//   ddy_ref_ = ref_states_matrix_(5, n);

//   yaw_ref_ = atan2(dy_ref_, dx_ref_);

//   // Reference Pose State
//   q_ref_ << x_ref_, y_ref_, yaw_ref_;

//   // Reference Velocities
//   vel_ref_ = sqrt(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);
//   omega_ref_ = (dx_ref_*ddy_ref_ - dy_ref_*ddx_ref_)/(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);
// }

void LinearControl::loadControllerParams() {
  ros::NodeHandle private_nh("~");

  std::string control_method = "LinearControl";
  private_nh.getParam(control_method + "/" + "const_gains", const_gains_);
  private_nh.getParam(control_method + "/" + "k_x", k_x_);
  private_nh.getParam(control_method + "/" + "k_y", k_y_);
  private_nh.getParam(control_method + "/" + "k_yaw", k_yaw_);
  private_nh.getParam(control_method + "/" + "g", g_);
  private_nh.getParam(control_method + "/" + "zeta", zeta_);
  private_nh.getParam(control_method + "/" + "vel_max_x", vel_max_);
  private_nh.getParam(control_method + "/" + "max_rot_vel", omega_max_);
}

void LinearControl::displayControllerInfo() {
  ROS_INFO("Linear Control Parameters:");

  if (const_gains_) {
    ROS_INFO("Constant gains values: ");
    ROS_INFO("Gain k_x: %2f", k_x_);
    ROS_INFO("Gain k_y: %2f", k_y_);
    ROS_INFO("Gain k_yaw: %2f", k_yaw_);
  } else {
    ROS_INFO("Design parameter g: %2f", g_);
    ROS_INFO("Design parameter zeta: %2f", zeta_);
  }

  ROS_INFO("Maximum linear velocity: %2f", vel_max_);
  ROS_INFO("Maximum angular velocity: %2f", omega_max_);

  if (const_trajectory_) {
    ROS_WARN("Set constant trajectory.");
  }
}

// void LinearControl::loadControllerParams() {
//   ros::NodeHandle private_nh("~");

//   std::string name = control_method_name_;
//   private_nh.getParam(name + "/" + "constant_gains", const_gains_);
//   private_nh.getParam(name + "/" + "k_x", k_x_);
//   private_nh.getParam(name + "/" + "k_y", k_y_);
//   private_nh.getParam(name + "/" + "k_yaw", k_yaw_);
//   private_nh.getParam(name + "/" + "g", g_);
//   private_nh.getParam(name + "/" + "zeta", zeta_);

//   if (g_ < 0.0) {
//     ROS_ERROR("Invalid design parameter.");
//     throw std::invalid_argument("The designer parameter g received a negative value.");
//   }
// }

// void LinearControl::displayControllerInfo() {}

// void LinearControl::updateReferenceState(double time) {
//   traj_gen_.updateReferenceState(time);
// }

// bool LinearControl::isGoalReached() {
//   return true;
// }

// void LinearControl::fillReferencePath(std::vector<std::pair<double, double>> *path) {
//   traj_gen_.fillReferencePath(path);
// }

void LinearControl::initializeMatrices() {
  q_ref_ = MatrixXd(3, 1);
  q_curr_ = MatrixXd(3, 1);
  tf_to_global_ = MatrixXd(3, 3);
  error_ = MatrixXd(3, 1);
}


}  // namespace trajectory_tracking_control
