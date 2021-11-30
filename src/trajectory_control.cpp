/*
  Copyright 2021 - Rafael Barreto
*/

#include "trajectory_tracking_control/trajectory_control.hpp"

namespace trajectory_tracking_control {


TrajectoryControl::TrajectoryControl(std::string action_name, ros::NodeHandle *nodehandle, tf2_ros::Buffer& tf_buffer)
  : action_name_(action_name),
    nh_(*nodehandle),
    tf_buffer_(tf_buffer),
    action_server_(nh_, action_name_, boost::bind(&TrajectoryControl::executeCB, this, _1), false) {
  action_server_.start();

  // Load common parameters
  loadCommonParams();

  // Initialize Publishers
  initializePublishers();
}

void TrajectoryControl::executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  // Set goal reached false
  goal_reached_ = false;

  // Velocity Command
  geometry_msgs::Twist cmd_vel;

  ros::Rate rate(10);

  // TODO(Rafael) delete
  const_trajectory_ = goal->const_trajectory;

  TrajectoryGenerator traj_gen(&nh_, goal->sampling_time);

  PoseHandler pose_handler(&tf_buffer_);

  // Make trajectory
  if (const_trajectory_) {
    traj_gen.makeConstantTrajectory();
  } else {
    // Set goal position
    goal_position_.x = goal->path.poses[goal->path.poses.size() - 1].position.x;
    goal_position_.y = goal->path.poses[goal->path.poses.size() - 1].position.y;

    traj_gen.makeTrajectory(goal->path, goal->average_velocity);
    goal_distance_ = traj_gen.getGoalDistance();
  }

  computeControlMethod(goal);

  switch (control_method_) {
    case ControlMethod::LINEAR:
      controller_ = new LinearControl(&traj_gen, &pose_handler);
      break;
    default:
      ROS_ERROR("No control method");
      break;
  }

  // ROS Time
  ros::Time zero_time = ros::Time::now();
  ros::Duration delta_t;
  double delta_t_sec;

  while (ros::ok() && !goal_reached_) {
    if (controller_->isGoalReached()) {
      goal_reached_ = true;
      continue;
    }

    delta_t = ros::Time::now() - zero_time;
    delta_t_sec = delta_t.toSec();

    // ROS_INFO("Time: %2f", delta_t_sec);


    controller_->updateReferenceState(delta_t_sec);

    if (controller_->computeVelocityCommands(cmd_vel)) {
      // Publish cmd_vel
      cmd_vel_pub_.publish(cmd_vel);
    } else {
      ROS_DEBUG("The controller could not find a valid velocity command.");
    }

    // Feedback
    feedback();

    rate.sleep();
  }

  if (goal_distance_) {
    result();
  }

  // Stop the robot
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_.publish(cmd_vel);
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
  controller_ = nullptr;
}

void TrajectoryControl::feedback() {}

void TrajectoryControl::result() {}

void TrajectoryControl::loadCommonParams() {
  ros::NodeHandle private_nh("~");

  private_nh.getParam("constant_trajectory", const_trajectory_);
  private_nh.getParam("vel_max_x", vel_max_);
  private_nh.getParam("max_rot_vel", omega_max_);
  private_nh.getParam("xy_goal_tolerance", xy_goal_tolerance_);
}

void TrajectoryControl::initializePublishers() {
  ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference_pose", 100, true);
  ref_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("reference_planner", 100, true);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);
  ref_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("reference_cmd_vel", 100, true);
}

}  // namespace trajectory_tracking_control
