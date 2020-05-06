// Copyright 2020

#include <trajectory_tracking_control/controller.h>

namespace trajectory_tracking_control {

Controller::Controller(const std::string &name, ros::NodeHandle *nodehandle) :
                       action_name_(name),
                       nh_(*nodehandle),
                       as_(nh_, name, boost::bind(&Controller::executeCB, this, _1), false) {
  as_.start();
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

  m_ref_states_ = MatrixXd(matrix_rows_size, matrix_columns_size);
  int index;
  for (int row = 0; row < matrix_rows_size; ++row) {
    for (int col = 0; col < matrix_columns_size; ++col) {
      index = row*matrix_columns_size + col;
      m_ref_states_(row, col) = ref_states_arr.data[index];
    }
  }
}
}  // namespace trajectory_tracking_control
