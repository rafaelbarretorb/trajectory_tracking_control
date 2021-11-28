/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_CONTROLLER_HPP_
#define TRAJECTORY_TRACKING_CONTROL_CONTROLLER_HPP_


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

#include "trajectory_tracking_control/pose_handler.hpp"
#include "trajectory_tracking_control/trajectory_generator.hpp"


using Eigen::MatrixXd;

namespace trajectory_tracking_control {

using ExecuteTrajectoryTrackingActionServer =
  actionlib::SimpleActionServer<trajectory_tracking_control::ExecuteTrajectoryTrackingAction>;


class Controller {
 public:
  Controller(std::string action_name, ros::NodeHandle *nodehandle, tf2_ros::Buffer& tf_buffer);  // NOLINT

  void executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal);

  bool isGoalReached();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel); //NOLINT

  virtual void updateReferenceState(int n);

  void loadControllerParams();

  void displayControllerInfo();

 protected:
  PoseHandler pose_handler_;
  std::string action_name_;
  ros::NodeHandle nh_;
  ExecuteTrajectoryTrackingActionServer action_server_;

  ExecuteTrajectoryTrackingFeedback feedback_;
  ExecuteTrajectoryTrackingResult result_;

  /*
    Reference States Matrix(6, n)

    | x_0    x_1    ...  x_n   |
    | y_0    y_1    ...  y_n   |
    | dx_0   dx_1   ...  dx_n  |
    | dy_0   dy_1   ...  dy_n  |
    | ddx_0  ddx_1  ...  ddx_n |
    | ddy_0  ddy_1  ...  ddy_n |

  */
  MatrixXd ref_states_matrix_;

  // Current Pose
  geometry_msgs::Pose curr_pose_;

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

  ros::ServiceClient ref_states_srv_;

  ros::Subscriber pose_sub_;

  geometry_msgs::Point goal_position_;

  ros::Publisher ref_pose_pub_;
  ros::Publisher ref_path_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher ref_cmd_vel_pub_;

  double goal_distance_;

  double vel_old_{0.0};

  bool goal_reached_{false};

  bool constant_gains_{false};

  bool constant_trajectory_{false};

  double xy_goal_tolerance_;

  // TODO(Rafael) remove and set it from action msg
  std::string controller_type_{std::string("Linear")};

  TrajectoryGenerator traj_gen_;

  geometry_msgs::Twist ref_cmd_vel_;
};
}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_CONTROLLER_HPP_
