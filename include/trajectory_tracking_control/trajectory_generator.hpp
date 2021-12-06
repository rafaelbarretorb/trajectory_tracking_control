/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_  // NOLINT
#define TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <eigen3/Eigen/Core>   // MatrixXd

#include <string>
#include <vector>
#include <utility>

// Service
#include "trajectory_tracking_control/ComputeReferenceStates.h"

using Eigen::MatrixXd;

namespace trajectory_tracking_control {

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(ros::NodeHandle *nodehandle, double sampling_time);

  void makeConstantTrajectory();

  void makeTrajectory(const geometry_msgs::PoseArray &path, double vel_avg);

  double getGoalDistance() const;

  void updateReferenceState(int discrete_time);

  void publishReferencePath();

  double getReferenceX() const;

  double getReferenceY() const;

  double getReferenceDX() const;

  double getReferenceDY() const;

  double getReferenceDDX() const;

  double getReferenceDDY() const;

  void initializePublishers();

 private:

  ros::ServiceClient ref_states_srv_;
  double goal_distance_;
  ros::NodeHandle nh_;
  MatrixXd ref_states_matrix_;
  double sampling_time_;

  // Current reference state
  double x_ref_, y_ref_, dx_ref_, dy_ref_, ddx_ref_, ddy_ref_;

  ros::Publisher ref_path_pub_;
};

}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_
