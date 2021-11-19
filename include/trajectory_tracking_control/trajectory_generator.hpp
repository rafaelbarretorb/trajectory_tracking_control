/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_  // NOLINT
#define TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <eigen3/Eigen/Core>   // MatrixXd

// Service
#include "trajectory_tracking_control/ComputeReferenceStates.h"

using Eigen::MatrixXd;

namespace trajectory_tracking_control {

class TrajectoryGenerator {
 public:
  TrajectoryGenerator() = default;

  void makeConstantTrajectory(MatrixXd &ref_states_matrix,  // NOLINT
                              double vel_avg,
                              double t_sampling) {
    // TODO(Rafael) remove hard coding
    double A = 3.0;
    double x_offset = 2.0;
    double y_offset = 0.0;
    double freq = 2*M_PI/60;
    int N = 150;

    double Ts = 0.5;
    double time;

    // Just two rows
    MatrixXd ref_states_matrix_(6, N);
    for (int i = 0; i < N; ++i) {
      time = i*Ts;
      ref_states_matrix_(0, i) = x_offset + A*sin(freq*time);
      ref_states_matrix_(1, i) = y_offset + A*sin(2*freq*time);
      ref_states_matrix_(2, i) = freq*A*cos(freq*time);
      ref_states_matrix_(3, i) = 2*freq*A*cos(2*freq*time);
      ref_states_matrix_(4, i) = -freq*freq*A*sin(freq*time);
      ref_states_matrix_(5, i) = -4*freq*freq*A*sin(2*freq*time);
    }
  }

  void makeTrajectory(const geometry_msgs::PoseArray &path,
                      double vel_avg,
                      double t_sampling,
                      MatrixXd &ref_states_matrix) {  // NOLINT
    // Request Service
    trajectory_tracking_control::ComputeReferenceStates srv;
    srv.request.path = path;
    srv.request.average_velocity = vel_avg;
    srv.request.sampling_time = t_sampling;

    std_msgs::Float32MultiArray ref_states_arr;
    int matrix_rows_size, matrix_columns_size;

    if (ref_states_srv_.call(srv)) {
      // get the service response
      ref_states_arr = srv.response.data;
      matrix_rows_size = srv.response.rows_size;
      matrix_columns_size = srv.response.columns_size;
      goal_distance_ = srv.response.goal_distance;

    } else {
      ROS_ERROR("Failed to call service Coverage Path Planning");
    }

    // Initialize the matrix
    ref_states_matrix = MatrixXd(matrix_rows_size, matrix_columns_size);

    int index;
    for (int row = 0; row < matrix_rows_size; ++row) {
      for (int col = 0; col < matrix_columns_size; ++col) {
        index = row*matrix_columns_size + col;
        ref_states_matrix(row, col) = ref_states_arr.data[index];
      }
    }
  }

 private:
  bool const_trajectory_{false};
  ros::ServiceClient ref_states_srv_;
  double goal_distance_;  // TODO(Rafael) DO I need this?
};

}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_
