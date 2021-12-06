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
#include <tf2_ros/transform_listener.h>

// Eigen
#include <eigen3/Eigen/Core>

// Action
#include <actionlib/server/simple_action_server.h>
#include <trajectory_tracking_control/ExecuteTrajectoryTrackingAction.h>

#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <utility>
#include <memory>

#include <boost/bind.hpp>

#include "trajectory_tracking_control/controller.hpp"

// Control Methods
#include "trajectory_tracking_control/linear_control.hpp"
// #include "trajectory_tracking_control/lyapunov_control.hpp"
// #include "trajectory_tracking_control/model_predictive_control.hpp"


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
  TrajectoryControl(std::string action_name, ros::NodeHandle *nodehandle, tf2_ros::Buffer * tf_buffer);

  void executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal);

  bool isGoalReached();


  void computeControlMethod(const ExecuteTrajectoryTrackingGoalConstPtr &goal);

  void actionFeedback();

  void actionResult();

  void initializePublishers();

  void initializeController(const ExecuteTrajectoryTrackingGoalConstPtr &goal);

  void updateReferenceState();

  void publishReferencePath();

  void publishReferencePose();

  void publishReferenceVelocity();

 protected:
  std::string action_name_;
  ros::NodeHandle nh_;
  ExecuteTrajectoryTrackingActionServer action_server_;

  ExecuteTrajectoryTrackingFeedback feedback_;
  ExecuteTrajectoryTrackingResult result_;

  ros::Subscriber pose_sub_;

  double goal_distance_;

  bool goal_reached_{false};

  tf2_ros::Buffer *tf_buffer_;

  ControlMethod control_method_;

  std::shared_ptr<Controller> controller_;

  bool goal_processing_fail_;

  ros::Time zero_time_;
  ros::Duration delta_t_;
  double delta_t_sec_;

  ros::Publisher cmd_vel_pub_;
};
}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_CONTROL_HPP_
