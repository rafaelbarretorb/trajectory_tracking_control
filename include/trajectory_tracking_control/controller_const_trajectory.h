// Copyright Rafael Barreto

#ifndef TRAJECTORY_TRACKING_CONTROL_CONTROLLER_CONST_TRAJECTORY_H_
#define TRAJECTORY_TRACKING_CONTROL_CONTROLLER_CONST_TRAJECTORY_H_

#include <ros/ros.h>

#include <trajectory_tracking_control/pose_handler.h>

// Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

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

#include <boost/bind.hpp>

using Eigen::MatrixXd;

namespace trajectory_tracking_control {


class ControllerConstTrajectory {
 public:
  ControllerConstTrajectory(ros::NodeHandle *nodehandle, tf2_ros::Buffer& tf);

  void execute();

 protected:
  PoseHandler pose_handler_;
  ros::NodeHandle nh_;

};
};  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_CONTROLLER_CONST_TRAJECTORY_H_ 
