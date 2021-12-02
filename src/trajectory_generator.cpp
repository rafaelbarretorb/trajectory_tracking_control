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

  // TODO remove hard code
  global_frame_ = "map";
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

  displayConstantTrajectoryInfo(x_offset, y_offset, x_amplitude, y_amplitude, freq);

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

  publishReferencePath();
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

  ROS_WARN("DEBUG: matrix_rows_size: %d", matrix_rows_size);
  ROS_WARN("DEBUG: matrix_columns_size: %d", matrix_columns_size);
  // Initialize the matrix
  ref_states_matrix_ = MatrixXd(matrix_rows_size, matrix_columns_size);

  int index;
  for (int row = 0; row < matrix_rows_size; ++row) {
    for (int col = 0; col < matrix_columns_size; ++col) {
      index = row*matrix_columns_size + col;
      ref_states_matrix_(row, col) = ref_states_arr.data[index];
    }
  }

  publishReferencePath();
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

double TrajectoryGenerator::getGoalDistance() const {
  return goal_distance_;
}

void TrajectoryGenerator::updateReferenceState(double time) {
  int n = round(time/(sampling_time_));
  if (n > ref_states_matrix_.cols() - 1) {
    n = ref_states_matrix_.cols() - 1;
  }

  x_ref_ = ref_states_matrix_(0, n);
  y_ref_ = ref_states_matrix_(1, n);
  dx_ref_ = ref_states_matrix_(2, n);
  dy_ref_ = ref_states_matrix_(3, n);
  ddx_ref_ = ref_states_matrix_(4, n);
  ddy_ref_ = ref_states_matrix_(5, n);

  yaw_ref_ = atan2(dy_ref_, dx_ref_);

  // Reference Pose State
  q_ref_ << x_ref_, y_ref_, yaw_ref_;

  // Reference Velocities
  vel_ref_ = sqrt(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);
  omega_ref_ = (dx_ref_*ddy_ref_ - dy_ref_*ddx_ref_)/(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);

  publishReferencePose();
  publishReferenceVelocity();
}

void TrajectoryGenerator::publishReferencePath() {
  geometry_msgs::PoseArray path;

  for (int i = 0; i < ref_states_matrix_.cols(); ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = ref_states_matrix_(0, i);
    pose.position.y = ref_states_matrix_(1, i);
    path.poses.push_back(pose);
  }

  ref_path_pub_.publish(path);
}


void TrajectoryGenerator::publishReferencePose() {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.pose.position.x = x_ref_;
  pose.pose.position.y = y_ref_;
  pose.pose.position.z = 0.0;

  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;

  quat_tf.setRPY(0, 0, yaw_ref_);
  quat_tf.normalize();
  quat_msg = tf2::toMsg(quat_tf);

  // Set orientation
  pose.pose.orientation = quat_msg;
  ref_pose_pub_.publish(pose);
}

void TrajectoryGenerator::publishReferenceVelocity() {
  // Publish reference velocity
  ref_cmd_vel_.linear.x = vel_ref_;
  ref_cmd_vel_.angular.z = omega_ref_;
  ref_cmd_vel_pub_.publish(ref_cmd_vel_);
}

void TrajectoryGenerator::initializePublishers() {
  ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference_pose", 100, true);
  ref_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("reference_planner", 100, true);
  ref_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("reference_cmd_vel", 100, true);
}


}  // namespace trajectory_tracking_control
