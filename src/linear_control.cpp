/*
  Copyright 2021 - Rafael Barreto
*/

#include "trajectory_tracking_control/linear_control.hpp"

namespace trajectory_tracking_control {

LinearControl::LinearControl(ros::NodeHandle *nodehandle,
                             const ExecuteTrajectoryTrackingGoalConstPtr &goal) :
                             nh_(*nodehandle),
                             const_trajectory_(goal->const_trajectory),
                             path_(goal->path),
                             make_trajectory_(goal->make_trajectory),
                             avg_velocity_(goal->average_velocity),
                             sampling_time_(goal->sampling_time),
                             traj_gen_(&nh_, sampling_time_) {
  // TODO(Rafael) remove hard code
  global_frame_ = "map";
  
  loadControllerParams();

  initializePublishers();

  displayControllerInfo();

  initializeMatrices();

  makeTrajectory();
}

bool LinearControl::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  // double omega;
  // double vel;

  // // Update robot pose
  // curr_pose_ = pose_handler_.getRobotPose();
  // yaw_curr_ = pose_handler_.getYawFromQuaternion(curr_pose_.orientation);
  // q_curr_ << curr_pose_.position.x, curr_pose_.position.y, yaw_curr_;


  // // Transform to global coordinate
  // tf_to_global_ << cos(yaw_curr_), sin(yaw_curr_), 0.0,
  //                  -sin(yaw_curr_), cos(yaw_curr_), 0.0,
  //                  0.0, 0.0, 1.0;

  // // Pose Error
  // error_ = tf_to_global_ * (q_ref_ - q_curr_);

  // // Wrap to PI yaw error
  // error_(2, 0) = angles::normalize_angle(error_(2, 0));

  // // Control
  // error_x_ = error_(0, 0);
  // error_y_ = error_(1, 0);
  // error_yaw_ = error_(2, 0);

  // if (!const_gains_) {
  //   // Variable gains
  //   k_x_ = 2*zeta_*sqrt(omega_ref_*omega_ref_ + g_*vel_ref_*vel_ref_);
  //   k_y_ = g_*vel_ref_;
  //   k_yaw_ = k_x_;
  // }

  // // Compute Velocities
  // vel = vel_ref_*cos(error_yaw_) + k_x_*error_x_;
  // omega = omega_ref_ + k_y_*error_y_ + k_yaw_*error_yaw_;

  // // Ensure that the angular velocity does not exceed the maximum allowed
  // if (fabs(omega) > omega_max_) {
  //   omega = copysign(omega_max_, omega);
  // }

  // if (vel < 0.0) {
  //   vel = 0.0;
  // }

  // // Ensure that the linear velocity does not exceed the maximum allowed
  // if (vel > vel_max_) {
  //   vel = vel_max_;
  // }

  // cmd_vel.linear.x = vel;
  // cmd_vel.angular.z = omega;

  return true;
}

void LinearControl::makeTrajectory() {

  if (const_trajectory_) {
    traj_gen_.makeConstantTrajectory();
  } else {
    // Set goal position
    goal_position_.x = path_.poses[path_.poses.size() - 1].position.x;
    goal_position_.y = path_.poses[path_.poses.size() - 1].position.y;

    traj_gen_.makeTrajectory(path_, avg_velocity_);
    goal_distance_ = traj_gen_.getGoalDistance();
  }
}

void LinearControl::updateReferenceState(double time) {
  int n = round(time/(sampling_time_));

  traj_gen_.updateReferenceState(n);

  x_ref_ = traj_gen_.getReferenceX();
  y_ref_ = traj_gen_.getReferenceY();
  dx_ref_ = traj_gen_.getReferenceDX();
  dy_ref_ = traj_gen_.getReferenceDY();
  ddx_ref_ = traj_gen_.getReferenceDDX();
  ddy_ref_ = traj_gen_.getReferenceDDY();

  yaw_ref_ = atan2(dy_ref_, dx_ref_);

  // Reference Pose State
  q_ref_ << x_ref_, y_ref_, yaw_ref_;

  // Reference Velocities
  vel_ref_ = sqrt(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);
  omega_ref_ = (dx_ref_*ddy_ref_ - dy_ref_*ddx_ref_)/(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);
}

void LinearControl::loadCommonParams() {
  ros::NodeHandle private_nh("~");

  private_nh.getParam("vel_max_x", vel_max_);
  private_nh.getParam("max_rot_vel", omega_max_);
  private_nh.getParam("xy_goal_tolerance", xy_goal_tolerance_);
}

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

void LinearControl::initializeMatrices() {
  q_ref_ = MatrixXd(3, 1);
  q_curr_ = MatrixXd(3, 1);
  tf_to_global_ = MatrixXd(3, 3);
  error_ = MatrixXd(3, 1);
}


void LinearControl::publishReferencePath() {
  traj_gen_.publishReferencePath();
}

void LinearControl::publishReferencePose() {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.pose.position.x = x_ref_;
  pose.pose.position.y = y_ref_;
  pose.pose.position.z = 0.0;

  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;

  quat_tf.setRPY(0, 0, yaw_ref_);
  quat_tf.normalize();
  quat_msg = tf2::toMsg(quat_tf);

  // Set orientation
  pose.pose.orientation = quat_msg;
  ref_pose_pub_.publish(pose);
}

bool LinearControl::isGoalReached() {
  return (euclideanDistance2D(goal_position_.x,
                              goal_position_.y,
                              curr_pose_.position.x,
                              curr_pose_.position.y) < xy_goal_tolerance_) ? true : false;
}

// void TrajectoryControlROS::publishReferenceVelocity() {
//   ref_cmd_vel_.linear.x = controller_->getReferenceLinearVelocity();
//   ref_cmd_vel_.angular.z = controller_->getReferenceAngularVelocity();
//   ref_cmd_vel_pub_.publish(ref_cmd_vel_);
// }

void LinearControl::initializePublishers() {
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
  ref_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("reference_cmd_vel", 10, true);
  ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference_pose", 10, true);
}

}  // namespace trajectory_tracking_control
