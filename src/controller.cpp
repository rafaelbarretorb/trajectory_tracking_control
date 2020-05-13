// Copyright 2020

#include <trajectory_tracking_control/controller.h>

namespace trajectory_tracking_control {

Controller::Controller(const std::string &name, ros::NodeHandle *nodehandle) :
                       action_name_(name),
                       nh_(*nodehandle),
                       as_(nh_, name, boost::bind(&Controller::executeCB, this, _1), false) {
  as_.start();

  // Initialize matrices

  q_ref_ = MatrixXd(3, 1);
  q_curr_ = MatrixXd(3, 1);
  tf_to_global_ = MatrixXd(3, 3);
  error_ = MatrixXd(3, 1);

  pose_sub_ = nh_.subscribe("/odometry/filtered", 100, &Controller::updateCurrentPoseCB, this);

  ref_pose_pub_ = nh_.advertise<"reference_pose">()
}

void Controller::executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  // Set goal position
  goal_position_.x = goal->path.poses[goal->path.poses.size() - 1].position.x;
  goal_position_.y = goal->path.poses[goal->path.poses.size() - 1].position.y;

  // Request Reference Matrix
  requestReferenceMatrix(goal->path, goal->velocity_average, goal->sampling_time);

  while (ros::ok() && !isGoalReached()) {

  //   if (computeVelocityCommands()) {
  //     // Publish cmd_vel

  //   } else {
  //     ROS_DEBUG("The controller could not find a valid plan.");
  //   }

  }
}

bool Controller::isGoalReached() {
  double distance = euclideanDistance(goal_position_.x, goal_position_.y,
                                      robot_pose_.position.x, robot_pose_.position.y);
  // TODO(BARRETO) remove after, hard coding
  double tolerance = 0.2;

  if (distance < tolerance) {
    return true;
  } else {
    return false;
  }
}

void Controller::updateCurrentPoseCB(const nav_msgs::Odometry & msg) {
  robot_pose_.position.x = msg.pose.pose.position.x;
  robot_pose_.position.y = msg.pose.pose.position.y;

  robot_pose_.orientation.x = msg.pose.pose.orientation.x;
  robot_pose_.orientation.y = msg.pose.pose.orientation.y;
  robot_pose_.orientation.z = msg.pose.pose.orientation.z;
  robot_pose_.orientation.w = msg.pose.pose.orientation.w;
}

bool Controller::computeVelocityCommands(geometry_msgs::Twist& cmd_vel, int time_idx) {
  double omega;
  double vel;

  x_ref_ = ref_states_matrix_(0, time_idx);
  y_ref_ = ref_states_matrix_(1, time_idx);
  dx_ref_ = ref_states_matrix_(2, time_idx);
  dy_ref_ = ref_states_matrix_(3, time_idx);
  ddx_ref_ = ref_states_matrix_(4, time_idx);
  ddy_ref_ = ref_states_matrix_(5, time_idx);

  yaw_ref_ = atan2(dy_ref_, dx_ref_);

  q_ref_ << x_ref_, y_ref_, yaw_ref_;

  // Publish reference pose
  // TODO(BARRETO)

  yaw_curr_ = getYawFromQuaternion(robot_pose_.orientation);

  q_curr_ << robot_pose_.position.x, robot_pose_.position.y, yaw_curr_;

  vel_ref_ = sqrt(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);


  tf_to_global_ << cos(yaw_curr_), sin(yaw_curr_), 0.0,
                   -sin(yaw_curr_), cos(yaw_curr_), 0.0,
                   0.0, 0.0, 1.0;

  // Posture Error
  error_ = tf_to_global_ * (q_curr_ - q_ref_);

  // Wrap to PI yaw error
  error_(2, 0) = angles::normalize_angle(error_(2, 0));

  // Control
  error_x_ = error_(0, 0);
  error_x_ = error_(1, 0);
  error_yaw_ = error_(2, 0);

  // TODO(BARRETO) Hard coding for now
  g_ = 1.5;
  zeta_ = 45;

  k_x_ = 2*zeta_*sqrt(omega_ref_*omega_ref_ + g_*vel_ref_*vel_ref_);
  k_y_ = g_*vel_ref_;
  k_yaw_ = k_x_;

  vel = vel_ref_*cos(error_yaw_) + k_x_*error_x_;
  omega = omega_ref_ + k_y_*error_y_ + k_yaw_*error_yaw_;

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

void Controller::getRobotPoseFromTF2() {}

double Controller::euclideanDistance(double x1, double y1, double x2, double y2) {
  return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

}  // namespace trajectory_tracking_control
