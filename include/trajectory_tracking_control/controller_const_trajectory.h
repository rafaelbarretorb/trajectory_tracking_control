// Copyright Rafael Barreto

#ifndef TRAJECTORY_TRACKING_CONTROL_CONTROLLER_CONST_TRAJECTORY_H_
#define TRAJECTORY_TRACKING_CONTROL_CONTROLLER_CONST_TRAJECTORY_H_

#include <ros/ros.h>

#include <trajectory_tracking_control/pose_handler.h>

// Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
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
using Eigen::VectorXf;

namespace trajectory_tracking_control {


class ControllerConstTrajectory {
 public:
  ControllerConstTrajectory(ros::NodeHandle *nodehandle, tf2_ros::Buffer& tf);

  void execute();

  void makeReferenceTrajectory(double freq);

  void computeReferenceStates(VectorXf *v, double time, double freq,
                             double x_offset, double y_offset, double A);

 protected:
  PoseHandler pose_handler_;
  ros::NodeHandle nh_;


  // Posture Error Matrix (3x1)
  MatrixXd error_;

  // Transform to global coordinate matrix (3x3)
  MatrixXd tf_to_global_;

  // Current Pose State (x, y, yaw)
  MatrixXd q_curr_;

  // Reference Pose State (x, y, yaw)
  MatrixXd q_ref_;

  // Pose Errors
  double error_x_, error_y_, error_yaw_;

  // Gains
  double k_x_, k_y_, k_yaw_;

  // Control design parameters
  double g_, zeta_;

  // Maximum absolute linear and angular velocities
  double vel_max_, omega_max_;

  // Current reference velocities
  double vel_ref_, omega_ref_;

  // Current reference state
  double x_ref_, y_ref_, dx_ref_, dy_ref_, ddx_ref_, ddy_ref_;

  // Yaw angle
  double yaw_ref_, yaw_curr_;

  // Publishers
  ros::Publisher ref_pose_pub_;
  ros::Publisher ref_path_pub_;
  ros::Publisher cmd_vel_pub_;
};
};  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_CONTROLLER_CONST_TRAJECTORY_H_ 
