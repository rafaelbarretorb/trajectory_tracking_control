/*
  Copyright 2021 - Rafael Barreto
*/

#include "trajectory_tracking_control/trajectory_control_ros.hpp"

namespace trajectory_tracking_control {

const std::map<int32_t, ControlMethod> ControlMethodMap::map = ControlMethodMap::create_map();

TrajectoryControlROS::TrajectoryControlROS(std::string action_name,
                                     ros::NodeHandle *nodehandle,
                                     tf2_ros::Buffer& tf_buffer) :
                                     action_name_(action_name),
                                     nh_(*nodehandle),
                                     tf_buffer_(tf_buffer),
                                     action_server_(nh_,
                                                    action_name_,
                                                    boost::bind(&TrajectoryControlROS::executeCB, this, _1),
                                                    false) {
  action_server_.start();

  loadCommonParams();

  initializePublishers();
}

void TrajectoryControlROS::executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  goal_reached_ = false;

  // Velocity Command
  geometry_msgs::Twist cmd_vel;

  // Loop rate
  ros::Rate rate(10);

  initializeController(goal);

  // Publish reference path
  // publishReferencePath();

  // ROS Time
  zero_time_ = ros::Time::now();

  // while (ros::ok() && !goal_reached_) {
  //   if (controller_->isGoalReached()) {
  //     goal_reached_ = true;
  //     continue;
  //   }

  //   // Update current reference state x, y, dx, dy, ddx, ddy
  //   // updateReferenceState();

  //   // Publish reference pose and velocity
  //   // publishReferencePose();
  //   // publishReferenceVelocity();

  //   // Compute the velocity command
  //   // if (controller_->computeVelocityCommands(cmd_vel)) {
  //   //   // Publish cmd_vel
  //   //   cmd_vel_pub_.publish(cmd_vel);
  //   // } else {
  //   //   ROS_DEBUG("The controller could not find a valid velocity command.");
  //   // }

  //   // Feedback Message
  //   actionFeedback();

  //   rate.sleep();
  // }

  // Result Message
  // actionResult();

  // Stop the robot
  // cmd_vel.linear.x = 0.0;
  // cmd_vel.angular.z = 0.0;
  // cmd_vel_pub_.publish(cmd_vel);

  // just for DEBUG
  actionResult();

}

void TrajectoryControlROS::computeControlMethod(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  if (ControlMethodMap::map.find(goal->control_method) == ControlMethodMap::map.end()) {
    // not found
    // TODO(Rafael) do something with this
    goal_processing_fail_ = true;
    ROS_ERROR("Failing processing control method");
  } else {
    // found
    control_method_ = ControlMethodMap::map.at(goal->control_method);
  }
}

TrajectoryControlROS::~TrajectoryControlROS() {
  // TODO smart pointer
  delete controller_;
  controller_ = nullptr;
}

// void TrajectoryControlROS::actionFeedback() {}

void TrajectoryControlROS::actionResult() {
  // result_.distance_traveled_percentage = feedback_.distance_traveled_percentage;
  result_.distance_traveled_percentage = 100;
  result_.mission_status = "SUCCEED";
  ROS_INFO("%s: Succeeded", action_name_.c_str());
  action_server_.setSucceeded(result_);
}

void TrajectoryControlROS::loadCommonParams() {
  ros::NodeHandle private_nh("~");

  private_nh.getParam("vel_max_x", vel_max_);
  private_nh.getParam("max_rot_vel", omega_max_);
  private_nh.getParam("xy_goal_tolerance", xy_goal_tolerance_);
}

void TrajectoryControlROS::initializePublishers() {
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);
  ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference_pose", 100, true);
  ref_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("reference_planner", 100, true);
  ref_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("reference_cmd_vel", 100, true);
}

void TrajectoryControlROS::initializeController(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  computeControlMethod(goal);

  switch (control_method_) {
    case ControlMethod::LINEAR:
      controller_ = new LinearControl(&nh_, goal);
      break;
    default:
      ROS_ERROR("No control method");
      break;
  }
}

// void TrajectoryControlROS::updateReferenceState() {
//   // Compute the time difference in seconds
//   delta_t_ = ros::Time::now() - zero_time_;
//   delta_t_sec_ = delta_t_.toSec();

//   // Update the reference state
//   controller_->updateReferenceState(delta_t_sec_);
// }

// void TrajectoryControlROS::publishReferencePath() {
//   std::vector<std::pair<double, double>> path;
//   geometry_msgs::PoseArray path_ros;

//   controller_->fillReferencePath(&path);

//   for (const auto &pair : path) {
//     geometry_msgs::Pose pose;
//     pose.position.x = pair.first;
//     pose.position.y = pair.second;
//     path_ros.poses.push_back(pose);
//   }

//   ref_path_pub_.publish(path);
// }

// void TrajectoryControlROS::publishReferencePose() {
//   geometry_msgs::PoseStamped pose;
//   pose.header.frame_id = global_frame_;
//   pose.pose.position.x = controller_->getReferenceX();
//   pose.pose.position.y = controller_->getReferenceY();
//   pose.pose.position.z = 0.0;

//   tf2::Quaternion quat_tf;
//   geometry_msgs::Quaternion quat_msg;

//   quat_tf.setRPY(0, 0, controller_->getReferenceYaw());
//   quat_tf.normalize();
//   quat_msg = tf2::toMsg(quat_tf);

//   // Set orientation
//   pose.pose.orientation = quat_msg;
//   ref_pose_pub_.publish(pose);
// }

// void TrajectoryControlROS::publishReferenceVelocity() {
//   ref_cmd_vel_.linear.x = controller_->getReferenceLinearVelocity();
//   ref_cmd_vel_.angular.z = controller_->getReferenceAngularVelocity();
//   ref_cmd_vel_pub_.publish(ref_cmd_vel_);
// }

}  // namespace trajectory_tracking_control
