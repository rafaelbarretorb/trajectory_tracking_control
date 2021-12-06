/*
  Copyright 2021 - Rafael Barreto
*/

#include "trajectory_tracking_control/trajectory_control.hpp"

namespace trajectory_tracking_control {

const std::map<int32_t, ControlMethod> ControlMethodMap::map = ControlMethodMap::create_map();

TrajectoryControl::TrajectoryControl(std::string action_name,
                                     ros::NodeHandle *nodehandle,
                                     tf2_ros::Buffer * tf_buffer) :
                                     action_name_(action_name),
                                     nh_(*nodehandle),
                                     tf_buffer_(tf_buffer),
                                     action_server_(nh_,
                                                    action_name_,
                                                    boost::bind(&TrajectoryControl::executeCB, this, _1),
                                                    false) {
  action_server_.start();

  initializePublishers();
}

void TrajectoryControl::executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  goal_reached_ = false;

  geometry_msgs::Twist cmd_vel;

  ros::Rate rate(10);

  initializeController(goal);

  publishReferencePath();

  // Start ROS Time
  zero_time_ = ros::Time::now();

  while (ros::ok() && !goal_reached_) {
    if (controller_->isGoalReached()) {
      goal_reached_ = true;
      continue;
    }

    // Update current reference state (x, y, dx, dy, ddx, ddy)
    updateReferenceState();

    // Publish reference pose and velocity
    publishReferencePose();
    publishReferenceVelocity();

    // Compute the velocity command
    if (controller_->computeVelocityCommands(cmd_vel)) {
      // Publish cmd_vel
      cmd_vel_pub_.publish(cmd_vel);
    } else {
      ROS_DEBUG("The controller could not find a valid velocity command.");
    }

    // Feedback Message
    actionFeedback();

    rate.sleep();
  }

  // Result Message
  actionResult();

  // Stop the robot
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_.publish(cmd_vel);
}

void TrajectoryControl::computeControlMethod(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
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

void TrajectoryControl::actionFeedback() {}

void TrajectoryControl::actionResult() {
  // result_.distance_traveled_percentage = feedback_.distance_traveled_percentage;
  result_.distance_traveled_percentage = 100;
  result_.mission_status = "SUCCEED";
  ROS_INFO("%s: Succeeded", action_name_.c_str());
  action_server_.setSucceeded(result_);
}

void TrajectoryControl::initializeController(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  computeControlMethod(goal);

  switch (control_method_) {
    case ControlMethod::LINEAR:
      controller_ = std::shared_ptr<Controller>(new LinearControl(&nh_, tf_buffer_, goal));
      break;
    default:
      ROS_ERROR("No control method");
      break;
  }
}

void TrajectoryControl::updateReferenceState() {
  // Compute the time difference in seconds
  delta_t_ = ros::Time::now() - zero_time_;
  delta_t_sec_ = delta_t_.toSec();

  // ROS_WARN("[DEBUG] delta_t_sec_ = %2f", delta_t_sec_);

  // Update the reference state
  controller_->updateReferenceState(delta_t_sec_);
}

void TrajectoryControl::publishReferencePath() {
  controller_->publishReferencePath();
}

void TrajectoryControl::publishReferencePose() {
  controller_->publishReferencePose();
}

void TrajectoryControl::initializePublishers() {
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
}

void TrajectoryControl::publishReferenceVelocity() {
  controller_->publishReferenceVelocity();
}

}  // namespace trajectory_tracking_control
