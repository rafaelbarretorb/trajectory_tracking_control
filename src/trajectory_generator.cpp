/*
  Copyright 2021 - Rafael Barreto
*/

#include <ros/ros.h>

#include "trajectory_tracking_control/trajectory_generator.hpp"


namespace trajectory_tracking_control {

void TrajectoryGenerator::makeConstantTrajectory(double vel_avg,  // TODO(RAFael) not using vel_avg
                            double t_sampling,
                            MatrixXd &ref_states_matrix) {  // NOLINT
  // TODO(Rafael) remove hard coding
  double A = 3.0;
  double x_offset = 1.0;
  double y_offset = 0.0;
  int N = 60;
  double freq = 2*M_PI/N;  // omega, not freq
  int m = N/t_sampling;

  double time;

  // Just two rows
  ref_states_matrix = MatrixXd(6, m);
  for (int i = 0; i < m; ++i) {
    time = i*t_sampling;
    ref_states_matrix(0, i) = x_offset + A*sin(freq*time);
    ref_states_matrix(1, i) = y_offset + A*sin(2*freq*time);
    ref_states_matrix(2, i) = freq*A*cos(freq*time);
    ref_states_matrix(3, i) = 2*freq*A*cos(2*freq*time);
    ref_states_matrix(4, i) = -freq*freq*A*sin(freq*time);
    ref_states_matrix(5, i) = -4*freq*freq*A*sin(2*freq*time);
  }
}

void TrajectoryGenerator::makeTrajectory(const geometry_msgs::PoseArray &path,
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

}  // namespace trajectory_tracking_control
