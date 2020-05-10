
// Copyright Rafael Barreto

#ifndef TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_ // NOLINT
#define TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_

#include <ros/ros.h>

// Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

#include <boost/bind.hpp>

using Eigen::MatrixXd;

namespace trajectory_tracking_control {

typedef actionlib::SimpleActionServer<trajectory_tracking_control::ExecuteTrajectoryTrackingAction>
  ExecuteTrajectoryTrackingActionServer;


class Controller {
 public:
  Controller(const std::string &name, ros::NodeHandle *nodehandle);

  void executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal);

  void requestReferenceMatrix(const geometry_msgs::PoseArray &path, double vel_avg, double t_sampling);

  bool isGoalReached();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel, int time_idx); //NOLINT

  void updateCurrentPose();

  double getYawFromQuaternion(const geometry_msgs::Quaternion & quat_msg);

 protected:
  std::string action_name_;
  ros::NodeHandle nh_;
  ExecuteTrajectoryTrackingActionServer as_;
  std::vector<geometry_msgs::Point> reference_trajectory_;

  // Reference Matrix
  MatrixXd ref_states_matrix_;

  // Current Pose
  geometry_msgs::Pose robot_pose_;

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

  // 
  ros::ServiceClient ref_states_srv_;
};
};  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_ NOLINT
