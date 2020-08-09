// Copyright Rafael Barreto

#ifndef TRAJECTORY_TRACKING_CONTROL_POSE_HANDLER_H_ // NOLINT
#define TRAJECTORY_TRACKING_CONTROL_POSE_HANDLER_H_

#include <ros/ros.h>

// ROS Messages
#include <geometry_msgs/Pose.h>

// Angles
#include <angles/angles.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
// Eigen
// #include <Eigen/Core>
#include <eigen3/Eigen/Core>

// Action
#include <actionlib/server/simple_action_server.h>
#include <trajectory_tracking_control/ExecuteTrajectoryTrackingAction.h>

// Service
#include <trajectory_tracking_control/ComputeReferenceStates.h>

#include <vector>
#include <string>
#include <cmath>

#include <boost/bind.hpp>

using Eigen::MatrixXd;

namespace trajectory_tracking_control {


class PoseHandler {
 public:
  PoseHandler(tf2_ros::Buffer* tf);

  const geometry_msgs::Pose &getRobotPose();

  double getYawFromQuaternion(const geometry_msgs::Quaternion & quat_msg);

  double euclideanDistance(double x1, double y1, double x2, double y2);

  void publishReferencePose(double x, double y, double yaw, const ros::Publisher &pub);

  void makeReferencePath();

 protected:

  const std::string odom_frame_;
  const std::string global_frame_;
  const std::string robot_base_frame_;

	// TF2
	tf2_ros::Buffer* tf_;
	tf2_ros::TransformListener tfl_;

	geometry_msgs::Pose robot_pose_;

};
};  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_POSE_HANDLER_H_ NOLINT
