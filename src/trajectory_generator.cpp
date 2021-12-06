/*
  Copyright 2021 - Rafael Barreto
*/


#include "trajectory_tracking_control/trajectory_generator.hpp"


namespace trajectory_tracking_control {

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle *nodehandle,
                                         double sampling_time) :
                                         nh_(*nodehandle),
                                         sampling_time_(sampling_time) {
  // Reference States Service
  ref_states_srv_ = nh_.serviceClient<trajectory_tracking_control::ComputeReferenceStates>("ref_states_srv");

  initializePublishers();
}

void TrajectoryGenerator::makeConstantTrajectory() {
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

  double omega = 2*M_PI*freq;
  int m = 1/(freq*sampling_time_);
  double time;

  ref_states_matrix_ = MatrixXd(6, m);
  for (int i = 0; i < m; ++i) {
    time = i * sampling_time_;

    ref_states_matrix_(0, i) = x_offset + x_amplitude*sin(omega*time);
    ref_states_matrix_(1, i) = y_offset + y_amplitude*sin(2*omega*time);
    ref_states_matrix_(2, i) = omega*x_amplitude*cos(omega*time);
    ref_states_matrix_(3, i) = 2*omega*y_amplitude*cos(2*omega*time);
    ref_states_matrix_(4, i) = -omega*omega*x_amplitude*sin(omega*time);
    ref_states_matrix_(5, i) = -4*omega*omega*y_amplitude*sin(2*omega*time);
  }
}

void TrajectoryGenerator::makeTrajectory(const geometry_msgs::PoseArray &path,
                    double vel_avg) {
  // Request Service
  trajectory_tracking_control::ComputeReferenceStates srv;
  srv.request.path = path;
  srv.request.average_velocity = vel_avg;
  srv.request.sampling_time = sampling_time_;

  std_msgs::Float32MultiArray ref_states_arr;
  int matrix_rows_size, matrix_columns_size;

  if (ref_states_srv_.call(srv)) {
    // get the service response
    ref_states_arr = srv.response.data;
    matrix_rows_size = srv.response.rows_size;
    matrix_columns_size = srv.response.columns_size;
    goal_distance_ = srv.response.goal_distance;

  } else {
    ROS_ERROR("Failed to call Reference States Service");
  }

  // Initialize the matrix
  ref_states_matrix_ = MatrixXd(matrix_rows_size, matrix_columns_size);

  int index;
  for (int row = 0; row < matrix_rows_size; ++row) {
    for (int col = 0; col < matrix_columns_size; ++col) {
      index = row*matrix_columns_size + col;
      ref_states_matrix_(row, col) = ref_states_arr.data[index];
    }
  }
}

double TrajectoryGenerator::getGoalDistance() const {
  return goal_distance_;
}

void TrajectoryGenerator::updateReferenceState(int discrete_time) {
  if (discrete_time > ref_states_matrix_.cols() - 1) {
    discrete_time = ref_states_matrix_.cols() - 1;
  }

  x_ref_ = ref_states_matrix_(0, discrete_time);
  y_ref_ = ref_states_matrix_(1, discrete_time);
  dx_ref_ = ref_states_matrix_(2, discrete_time);
  dy_ref_ = ref_states_matrix_(3, discrete_time);
  ddx_ref_ = ref_states_matrix_(4, discrete_time);
  ddy_ref_ = ref_states_matrix_(5, discrete_time);
}

void TrajectoryGenerator::publishReferencePath() {
  geometry_msgs::PoseArray path;

  int n = ref_states_matrix_.cols();

  for (int i = 0; i < ref_states_matrix_.cols(); ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = ref_states_matrix_(0, i);
    pose.position.y = ref_states_matrix_(1, i);
    path.poses.push_back(pose);
  }

  ref_path_pub_.publish(path);
}

double TrajectoryGenerator::getReferenceX() const {
  return x_ref_;
}

double TrajectoryGenerator::getReferenceY() const {
  return y_ref_;
}

double TrajectoryGenerator::getReferenceDX() const {
  return dx_ref_;
}

double TrajectoryGenerator::getReferenceDY() const {
  return dy_ref_;
}

double TrajectoryGenerator::getReferenceDDX() const {
  return ddx_ref_;
}

double TrajectoryGenerator::getReferenceDDY() const {
  return ddy_ref_;
}

void TrajectoryGenerator::initializePublishers() {
  ref_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("reference_planner", 100, true);
}

}  // namespace trajectory_tracking_control
