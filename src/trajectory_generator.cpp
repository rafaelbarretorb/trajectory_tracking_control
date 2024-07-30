/*
  Copyright 2021 - Rafael Barreto
*/



#include "trajectory_tracking_control/trajectory_generator.hpp"


namespace trajectory_tracking_control {


void TrajectoryGenerator::makeConstantTrajectory(double t_sampling, MatrixXd &ref_states_matrix) {   
  double x_offset = 1.1;
  double y_offset = 0.9;
  double x_amplitude = 0.7;
  double y_amplitude = 0.7;
  double freq = 0.02;

  // displayConstantTrajectoryInfo(x_offset, y_offset, x_amplitude, y_amplitude, freq);

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

// void TrajectoryGenerator::displayConstantTrajectoryInfo(double x_offset,
//                                                         double y_offset,
//                                                         double x_amp,
//                                                         double y_amp,
//                                                         double freq) {
//   ROS_INFO("Constant Trajectory parameters: ");
//   ROS_INFO("Offset X: %2f", x_offset);
//   ROS_INFO("Offset Y: %2f", y_offset);
//   ROS_INFO("Amplitude X: %2f", x_amp);
//   ROS_INFO("Amplitude Y: %2f", y_amp);
//   ROS_INFO("Frequency: %2f", freq);
// }



}  // namespace trajectory_tracking_control
