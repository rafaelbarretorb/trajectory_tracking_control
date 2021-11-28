/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_CONTROL_HPP_
#define TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_CONTROL_HPP_


// Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>



// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



// Action
#include <actionlib/server/simple_action_server.h>
#include <trajectory_tracking_control/ExecuteTrajectoryTrackingAction.h>

#include <vector>
#include <map>
#include <string>
#include <cmath>

#include <boost/bind.hpp>

#include "trajectory_tracking_control/pose_handler.hpp"
#include "trajectory_tracking_control/trajectory_generator.hpp"

#include "trajectory_tracking_control/controller.hpp"

// Control Methods
#include "trajectory_tracking_control/linear_control.hpp"
#include "trajectory_tracking_control/lyapunov_control.hpp"
#include "trajectory_tracking_control/model_predictive_control.hpp"



namespace trajectory_tracking_control {

enum class ControlMethod { LINEAR, LYAPUNOV, MPC };

struct ControlMethodMap {
  static std::map<int32_t, ControlMethod> create_map() {
    std::map<int32_t, ControlMethod> m;
    m[ExecuteTrajectoryTrackingGoal::LINEAR] = ControlMethod::LINEAR;
    m[ExecuteTrajectoryTrackingGoal::LYAPUNOV] = ControlMethod::LYAPUNOV;
    m[ExecuteTrajectoryTrackingGoal::MPC] = ControlMethod::MPC;
    return m;
  }
  static const std::map<int32_t, ControlMethod> map;
};

using ExecuteTrajectoryTrackingActionServer =
  actionlib::SimpleActionServer<trajectory_tracking_control::ExecuteTrajectoryTrackingAction>;


class TrajectoryControl {
 public:
  TrajectoryControl(std::string action_name, ros::NodeHandle *nodehandle, tf2_ros::Buffer& tf_buffer);  // NOLINT

  ~TrajectoryControl();

  void executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal);

  bool isGoalReached();

  virtual void updateReferenceState(int n);

  void computeControlMethod(const ExecuteTrajectoryTrackingGoalConstPtr &goal);

 protected:

  std::string action_name_;
  ros::NodeHandle nh_;
  ExecuteTrajectoryTrackingActionServer action_server_;

  ExecuteTrajectoryTrackingFeedback feedback_;
  ExecuteTrajectoryTrackingResult result_;





  ros::Subscriber pose_sub_;

  geometry_msgs::Point goal_position_;

  ros::Publisher ref_pose_pub_;
  ros::Publisher ref_path_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher ref_cmd_vel_pub_;

  double goal_distance_;

  double vel_old_{0.0}; // TODO(Rafael) check

  bool goal_reached_{false};

  

  bool constant_trajectory_{false};

  double xy_goal_tolerance_;

  // TODO(Rafael) remove and set it from action msg
  std::string controller_type_;

  TrajectoryGenerator traj_gen_;

  geometry_msgs::Twist ref_cmd_vel_;

  ControlMethod control_method_;

  Controller *controller_;

  bool goal_processing_fail_;



};
}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_CONTROLLER_HPP_