/*
  Copyright 2021 - Rafael Barreto
*/

#include "trajectory_tracking_control/trajectory_control.hpp"

namespace trajectory_tracking_control {


TrajectoryControl::TrajectoryControl(std::string action_name,
                       ros::NodeHandle *nodehandle,
                       tf2_ros::Buffer& tf_buffer) : action_name_(action_name),
                                                     nh_(*nodehandle),
                                                     pose_handler_(&tf_buffer),
                                                     traj_gen_(&nh_),
                                                     action_server_(nh_,
                                                                    action_name_,
                                                                    boost::bind(&TrajectoryControl::executeCB, this, _1),
                                                                    false) {
  action_server_.start();

  // Initialize matrices
  q_ref_ = MatrixXd(3, 1);
  q_curr_ = MatrixXd(3, 1);
  tf_to_global_ = MatrixXd(3, 3);
  error_ = MatrixXd(3, 1);

  // TODO(Rafael) create a new method for this
  // Initialize Publishers
  ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference_pose", 100, true);
  ref_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("reference_planner", 100, true);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);
  ref_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("reference_cmd_vel", 100, true);
}

void TrajectoryControl::executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  // Set goal reached false
  goal_reached_ = false;

  // Make trajectory
  if (goal->const_trajectory) {
    traj_gen_.makeConstantTrajectory(goal->sampling_time, ref_states_matrix_);
  } else {
    // Set goal position
    goal_position_.x = goal->path.poses[goal->path.poses.size() - 1].position.x;
    goal_position_.y = goal->path.poses[goal->path.poses.size() - 1].position.y;

    traj_gen_.makeTrajectory(goal->path, goal->average_velocity, goal->sampling_time, ref_states_matrix_);
    goal_distance_ = traj_gen_.getGoalDistance();
  }

  computeControlMethod(goal);

  switch (control_method_) {
    case ControlMethod::LINEAR:
      controller_ = new LinearControl;
      break;
    case ControlMethod::LYAPUNOV:
      controller_ = new LyapunovControl;
      break;
    case ControlMethod::MPC:
      controller_ = new MPC;
      break;
    default:
      // TODO(Rafael) Error
      break;
  }

  // Publish reference path
  pose_handler_.publishReferencePath(ref_states_matrix_, ref_path_pub_);

  geometry_msgs::Twist cmd_vel;
  int n;
  double step = goal_distance_/ref_states_matrix_.cols();

  // TODO(BARRETO) rate is not constant?
  ros::Rate rate(10);

  // ROS Time
  ros::Time zero_time = ros::Time::now();
  ros::Duration delta_t;
  double delta_t_sec, delta_t_finish;
  ros::Time zero_time2 = ros::Time::now();
  bool final = false;
}

bool TrajectoryControl::isGoalReached() {
  return (euclideanDistance2D(goal_position_.x,
                              goal_position_.y,
                              curr_pose_.position.x,
                              curr_pose_.position.y) < xy_goal_tolerance_) ? true : false;
}

void TrajectoryControl::updateReferenceState(int n) {
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

  // Publish reference pose
  pose_handler_.publishReferencePose(x_ref_, y_ref_, yaw_ref_, ref_pose_pub_);

  // Publish reference velocity
  ref_cmd_vel_.linear.x = vel_ref_;
  ref_cmd_vel_.angular.z = omega_ref_;
  ref_cmd_vel_pub_.publish(ref_cmd_vel_);
}


void TrajectoryControl::computeControlMethod(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  if (ControlMethodMap::map.find(goal->control_method) == ControlMethodMap::map.end()) {
    // not found
    goal_processing_fail_ = true;
    ROS_ERROR("Failing processing control method");
  } else {
    // found
    control_method_ = ControlMethodMap::map.at(goal->control_method);
  }
}

TrajectoryControl::~TrajectoryControl() {
  delete controller_;
  controller_{nullptr};
}

}  // namespace trajectory_tracking_control
