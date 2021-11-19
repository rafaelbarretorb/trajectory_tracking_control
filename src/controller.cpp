// Copyright 2020

#include "trajectory_tracking_control/controller.hpp"

namespace trajectory_tracking_control {


// TODO switct boost to std
Controller::Controller(const std::string &controller_type,
                       const std::string &action_name,
                       ros::NodeHandle *nodehandle,
                       tf2_ros::Buffer& tf) : controller_type_(controller_type),
                                              action_name_(action_name),
                                              nh_(*nodehandle),
                                              pose_handler_(&tf), 
                                              action_server_(nh_,
                                                             action_name_,
                                                             boost::bind(&Controller::executeCB, this, _1), false) {
  action_server_.start();

  //Load controller parameters
  loadControllerParams();

  // Display controller info
  displayControllerInfo();
  
  // Initialize matrices
  q_ref_ = MatrixXd(3, 1);
  q_curr_ = MatrixXd(3, 1);
  tf_to_global_ = MatrixXd(3, 3);
  error_ = MatrixXd(3, 1);

  // TODO create a new method for this
  // Initialize Publishers
  ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference_pose", 100, true);
  ref_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("reference_planner", 100, true);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);

  // Reference States Service
  ref_states_srv_ = nh_.serviceClient<trajectory_tracking_control::ComputeReferenceStates>("ref_states_srv");
}

void Controller::executeCB(const ExecuteTrajectoryTrackingGoalConstPtr &goal) {
  // Set goal reached false
  goal_reached_ = false;
  // Set goal position
  goal_position_.x = goal->path.poses[goal->path.poses.size() - 1].position.x;
  goal_position_.y = goal->path.poses[goal->path.poses.size() - 1].position.y;

  // Request Reference Matrix
  requestReferenceMatrix(goal->path, goal->average_velocity, goal->sampling_time);

  // Publish Reference path TODO(Rafael) here?
  pose_handler_.publishReferencePath(ref_states_matrix_, ref_path_pub_);

  geometry_msgs::Twist cmd_vel;
  int n;
  double step = goal_distance_/ref_states_matrix_.cols();

  // TODO(BARRETO) rate is not constant?
  ros::Rate rate(10);

  // ROS Time
  ros::Time zero_time = ros::Time::now();
  ros::Duration delta_t;
  double delta_t_sec, delta_t_finish;
  ros::Time zero_time2 = ros::Time::now();
  bool final = false;

  while (ros::ok() && !goal_reached_) {
    isGoalReached();

    delta_t = ros::Time::now() - zero_time;
    delta_t_sec = delta_t.toSec();

    // ROS_INFO("Time: %2f", delta_t_sec);

    n = round(delta_t_sec/(goal->sampling_time));
    if (n > ref_states_matrix_.cols() - 1) {
      n = ref_states_matrix_.cols() - 1;

      if (!final) {
        zero_time2 = ros::Time::now();
        final = true;
      }

      // Tmp: remove this
      delta_t_finish = (ros::Time::now() - zero_time2).toSec();
      if (delta_t_finish > 5.0)
        goal_reached_ = true;
    }

    updateReferenceState(n);
    if (computeVelocityCommands(cmd_vel)) {
      // Publish cmd_vel
      cmd_vel_pub_.publish(cmd_vel);

    } else {
      ROS_DEBUG("The controller could not find a valid velocity command.");
    }

    // Feedback
    feedback_.mission_status = "PROGRESS";
    feedback_.distance_traveled_percentage = 100*n*step/goal_distance_;
    action_server_.publishFeedback(feedback_);

    rate.sleep();
  }

  if (goal_distance_) {
    result_.distance_traveled_percentage = feedback_.distance_traveled_percentage;
    result_.mission_status = "SUCCEED";
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    action_server_.setSucceeded(result_);
  }

  // Stop the robot
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  // cmd_vel_pub_.publish(cmd_vel);
}

bool Controller::isGoalReached() {
  return (euclideanDistance2D(goal_position_.x,
                              goal_position_.y,
                              curr_pose_.position.x,
                              curr_pose_.position.y) < xy_goal_tolerance_) ? true : false;
}

void Controller::updateReferenceState(int n) {
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

  // Publish reference pose
  pose_handler_.publishReferencePose(x_ref_, y_ref_, yaw_ref_, ref_pose_pub_);
}

bool Controller::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  double omega;
  double vel;

  // Update robot pose
  curr_pose_ = pose_handler_.getRobotPose();
  yaw_curr_ = pose_handler_.getYawFromQuaternion(curr_pose_.orientation);
  q_curr_ << curr_pose_.position.x, curr_pose_.position.y, yaw_curr_;


  // Transform to global coordinate
  tf_to_global_ << cos(yaw_curr_), sin(yaw_curr_), 0.0,
                   -sin(yaw_curr_), cos(yaw_curr_), 0.0,
                   0.0, 0.0, 1.0;

  // Posture Error
  error_ = tf_to_global_ * (q_ref_ - q_curr_);

  // Wrap to PI yaw error
  error_(2, 0) = angles::normalize_angle(error_(2, 0));

  // Control
  error_x_ = error_(0, 0);
  error_y_ = error_(1, 0);
  error_yaw_ = error_(2, 0);

  if (!constant_gains_) {
    // Variable gains
    k_x_ = 2*zeta_*sqrt(omega_ref_*omega_ref_ + g_*vel_ref_*vel_ref_);
    k_y_ = g_*vel_ref_;
    k_yaw_ = k_x_;
  }

  // Compute Velocities
  vel = vel_ref_*cos(error_yaw_) + k_x_*error_x_;
  omega = omega_ref_ + k_y_*error_y_ + k_yaw_*error_yaw_;

  // Ensure that the angular velocity does not exceed the maximum allowed
  if (fabs(omega) > omega_max_) {
    omega = copysign(omega_max_, omega);
  }

  if (vel < 0.0){
    vel = 0.0;
  }
  
  // Ensure that the linear velocity does not exceed the maximum allowed
  if (vel > vel_max_) {
    vel = vel_max_;
  }

  cmd_vel.linear.x = vel;
  cmd_vel.angular.z = omega;

  return true;
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
    goal_distance_ = srv.response.goal_distance;

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

// TODO Create a new class called LinearController
void Controller::loadControllerParams() {
  ros::NodeHandle private_nh("~");

  private_nh.getParam(controller_type_ + "/" + "constant_gains", constant_gains_);
  private_nh.getParam(controller_type_ + "/" + "k_x", k_x_);
  private_nh.getParam(controller_type_ + "/" + "k_y", k_y_);
  private_nh.getParam(controller_type_ + "/" + "k_yaw", k_yaw_);
  private_nh.getParam(controller_type_ + "/" + "g", g_);
  private_nh.getParam(controller_type_ + "/" + "zeta", zeta_);
  private_nh.getParam(controller_type_ + "/" + "vel_max_x", vel_max_);
  private_nh.getParam(controller_type_ + "/" + "max_rot_vel", omega_max_);
  private_nh.getParam(controller_type_ + "/" + "constant_trajectory", constant_trajectory_);
  private_nh.getParam(controller_type_ + "/" + "xy_goal_tolerance", xy_goal_tolerance_);

  if (g_ < 0.0) {
    ROS_ERROR("Invalid design parameter.");
    throw std::invalid_argument( "The designer parameter g received a negative value.");
  }
}

void Controller::displayControllerInfo() {
  ROS_INFO("%s Controller Parameters: ", controller_type_.c_str());

  if (constant_gains_) {
    ROS_INFO("Constant gains values: ");
    ROS_INFO("Gain k_x: %2f", k_x_);
    ROS_INFO("Gain k_y: %2f", k_y_);
    ROS_INFO("Gain k_yaw: %2f", k_yaw_);
  } else {
    ROS_INFO("Design parameter g: %2f", g_);
    ROS_INFO("Design parameter zeta: %2f", zeta_);
  }

  ROS_INFO("Maximum linear velocity: %2f", vel_max_);
  ROS_INFO("Maximum angular velocity: %2f", omega_max_);
  ROS_INFO("Goal xy tolerance: %2f", xy_goal_tolerance_);

  if (constant_trajectory_) {
    ROS_WARN("Set constant trajectory.");
  }
}

}  // namespace trajectory_tracking_control
