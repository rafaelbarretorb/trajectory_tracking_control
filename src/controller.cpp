// Copyright 2020

#include <trajectory_tracking_control/controller.h>

namespace trajectory_tracking_control {

Controller::Controller(const std::string &name, ros::NodeHandle *nodehandle) :
                       action_name_(name),
                       nh_(*nodehandle),
                       as_(nh_, name, boost::bind(&Controller::executeCB, this, _1), false) {
  as_.start();

  // Initialize matrices

  // Posture error e(t)
  MatrixXd

}

void Controller::executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  requestReferenceMatrix(goal->path, goal->velocity_average, goal->sampling_time);

  while (ros::ok() || isGoalReached()) {

  //   if (computeVelocityCommands()) {
  //     // Publish cmd_vel

  //   } else {
  //     ROS_DEBUG("The controller could not find a valid plan.");
  //   }

  }
}

bool Controller::isGoalReached() {
  // TODO(BARRETO)
  return false;
}

void Controller::updateCurrentPose() {}

bool Controller::computeVelocityCommands(geometry_msgs::Twist& cmd_vel, int time_idx) {
  double omega;
  double vel;

  q_curr_ << robot_pose_.position.x, robot_pose_.position.y, getYawFromQuaternion(robot_pose_.orientation);

  // TODO(BARRETO) Hard coding for now
  g_ = 1.5;
  zeta_ = 45;


  

  vel = vel_ref_ * cos(error_yaw_) + k_x_*error_x_;
  omega = omega_ref_ + k_y_ * error_y_ + k_yaw_ * error_yaw_;

  // Ensure that the linear velocity does not exceed the maximum allowed
  if (vel > vel_max_) {
    vel = vel_max_;
  }

  // Ensure that the angular velocity does not exceed the maximum allowed
  if (fabs(omega) > omega_max_) {
    omega = copysign(omega_max_, omega);
  }

  cmd_vel.linear.x = vel;
  cmd_vel.angular.z = omega;
}

void Controller::requestReferenceMatrix(const geometry_msgs::PoseArray &path, double vel_avg, double t_sampling) {
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

  } else {
    ROS_ERROR("Failed to call service Coverage Path Planning");
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

double Controller::getYawFromQuaternion(const geometry_msgs::Quaternion & quat_msg) {
  double roll, pitch, yaw;

  tf2::Quaternion quat(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  tf2::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);

  return yaw;
}
}  // namespace trajectory_tracking_control
