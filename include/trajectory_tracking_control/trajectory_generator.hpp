/*
  
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_

#include <eigen3/Eigen/Core>   // MatrixXd

namespace trajectory_tracking_control {

using Eigen::MatrixXd;

class TrajectoryGenerator {
 public:
  void makeConstantTrajectory(double t_sampling, MatrixXd &ref_states_matrix);

 private:
//   void displayConstantTrajectoryInfo(double x_offset, double y_offset, double x_amp, double y_amp, double freq);
};

}  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_TRAJECTORY_GENERATOR_HPP_
