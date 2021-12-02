/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_POSE_HANDLER_HPP_
#define TRAJECTORY_TRACKING_CONTROL_POSE_HANDLER_HPP_

#include <ros/ros.h>

// ROS Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// Angles
#include <angles/angles.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include <eigen3/Eigen/Core>

#include <vector>
#include <string>
#include <cmath>


using Eigen::MatrixXd;


namespace trajectory_tracking_control {

inline float euclideanDistance2D(float x1, float y1, float x2, float y2) { return std::hypot((x1 - x2), (y1 - y2)); }

class PoseHandler {
 public:
  explicit PoseHandler(tf2_ros::Buffer* tf);

  const geometry_msgs::Pose &getRobotPose();

  double getYawFromQuaternion(const geometry_msgs::Quaternion & quat_msg);

 protected:
  std::string odom_frame_;
  std::string global_frame_;
  std::string robot_base_frame_;

  // TF2
  tf2_ros::Buffer* tf_;
  tf2_ros::TransformListener tfl_;

  geometry_msgs::Pose robot_pose_;
  geometry_msgs::Point goal_position_;

};
}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_POSE_HANDLER_HPP_ NOLINT
