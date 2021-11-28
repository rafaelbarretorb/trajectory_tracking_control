/*
  Copyright 2021 - Rafael Barreto
*/

#include <ros/ros.h>

#include "trajectory_tracking_control/trajectory_generator.hpp"


namespace trajectory_tracking_control {

void TrajectoryGenerator::makeConstantTrajectory(double t_sampling, MatrixXd &ref_states_matrix) {  // NOLINT
  ros::NodeHandle private_nh("~");
  
  double x_offset;
  double y_offset;
  double x_amplitude;
  double y_amplitude;
  double freq;

  private_nh.getParam("x_offset", x_offset);
  private_nh.getParam("y_offset", y_offset);
  private_nh.getParam("x_amplitude", x_amplitude);
  private_nh.getParam("y_amplitude", y_amplitude);
  private_nh.getParam("frequency", freq);

  displayConstantTrajectoryInfo(x_offset, y_offset, x_amplitude, y_amplitude, freq);

  double omega = 2*M_PI*freq;
  int m = 1/(freq*t_sampling);
  double time;

  ref_states_matrix = MatrixXd(6, m);
  for (int i = 0; i < m; ++i) {
    time = i*t_sampling;
    ref_states_matrix(0, i) = x_offset + x_amplitude*sin(omega*time);
    ref_states_matrix(1, i) = y_offset + y_amplitude*sin(2*omega*time);
    ref_states_matrix(2, i) = omega*x_amplitude*cos(omega*time);
    ref_states_matrix(3, i) = 2*omega*y_amplitude*cos(2*omega*time);
    ref_states_matrix(4, i) = -omega*omega*x_amplitude*sin(omega*time);
    ref_states_matrix(5, i) = -4*omega*omega*y_amplitude*sin(2*omega*time);
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

void TrajectoryGenerator::displayConstantTrajectoryInfo(double x_offset,
                                                        double y_offset,
                                                        double x_amp,
                                                        double y_amp,
                                                        double freq) {
  ROS_INFO("Constant Trajectory parameters: ");
  ROS_INFO("Offset X: %2f", x_offset);
  ROS_INFO("Offset Y: %2f", y_offset);
  ROS_INFO("Amplitude X: %2f", x_amp);
  ROS_INFO("Amplitude Y: %2f", y_amp);
  ROS_INFO("Frequency: %2f", freq);
}



}  // namespace trajectory_tracking_control
