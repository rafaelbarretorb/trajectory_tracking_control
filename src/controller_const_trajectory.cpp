#include <trajectory_tracking_control/controller_const_trajectory.h>

namespace trajectory_tracking_control {

ControllerConstTrajectory::ControllerConstTrajectory(ros::NodeHandle *nodehandle,
                                                      tf2_ros::Buffer& tf) : 
                                                      nh_(*nodehandle),
                                                      pose_handler_(&tf) {

  // Initialize Publishers
  ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference_pose", 100, true);
  ref_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("reference_planner", 100, true);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);

  // Initialize Matrices
  q_ref_ = MatrixXd(3, 1);
  q_curr_ = MatrixXd(3, 1);
  tf_to_global_ = MatrixXd(3, 3);
  error_ = MatrixXd(3, 1);

  vel_max_ = 0.55;
  omega_max_ = 0.5;

  // Just the loop function to test the localization for now
  execute();
}

void ControllerConstTrajectory::execute() {
  ros::Rate rate(40);

  ros::Duration(5.0).sleep();
  
  geometry_msgs::Twist cmd_vel;

  // Posture Error Matrix (3x1)
  MatrixXd error(3, 1);

  // Transform to global coordinate matrix (3x3)
  MatrixXd tf_to_global(3, 3);

  // Current Pose State (x, y, yaw)
  MatrixXd q_curr(3, 1);

  // Reference Pose State (x, y, yaw)
  MatrixXd q_ref(3, 1);

  // ROS Time
  ros::Time zero_time = ros::Time::now();
  ros::Duration delta_t;
  double delta_t_sec;

  double omega;
  double vel;

  // Max linear and angular velocities
  double vel_max = 0.3;
  double omega_max = 0.3;
  
  // Current pose of the robot
  geometry_msgs::Pose curr_pose;

  double A = 3.0;
  double x_offset = 2.0;
  double y_offset = 0.0;
  double freq = 2*M_PI/60;

  makeReferenceTrajectory(freq, x_offset, y_offset, A);
  pose_handler_.publishReferencePath(ref_states_matrix_, ref_path_pub_);

  // VectorXd ref_states(6);

  while (ros::ok()) {
    // Get time in secs
    delta_t = ros::Time::now() - zero_time;
    delta_t_sec = delta_t.toSec();

    computeReferenceStates(delta_t_sec, freq, x_offset, y_offset, A);
  
    // Update robot pose
    curr_pose = pose_handler_.getRobotPose();

    yaw_ref_ = atan2(dy_ref_, dx_ref_);

    q_ref_ << x_ref_, y_ref_, yaw_ref_;

    // Publish reference pose
    // ROS_INFO("Pose: (%f , %f)", x_ref_, y_ref_);
    pose_handler_.publishReferencePose(x_ref_, y_ref_, yaw_ref_, ref_pose_pub_);

    yaw_curr_ = pose_handler_.getYawFromQuaternion(curr_pose.orientation);

    q_curr_ << curr_pose.position.x, curr_pose.position.y, yaw_curr_;

    vel_ref_ = sqrt(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);

    omega_ref_ = (dx_ref_*ddy_ref_ - dy_ref_*ddx_ref_)/(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);

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

    // ROS_INFO("error_yaw = %f", vel);

    // TODO(BARRETO) Hard coding for now
    // g_ = 1.5;
    // zeta_ = 45;
    // g_ = 2.0;
    // zeta_ = 60;
    g_ = 1.0;
    zeta_ = 25;
    // k_x_ = 2*zeta_*sqrt(omega_ref_*omega_ref_ + g_*vel_ref_*vel_ref_);
    // k_y_ = g_*vel_ref_;
    // k_yaw_ = k_x_;

    // Constants gains
    k_x_ = 5;
    k_y_ = 25;
    k_yaw_ = 2*zeta_*sqrt(omega_ref_*omega_ref_ + g_*vel_ref_*vel_ref_);
    vel = vel_ref_*cos(error_yaw_) + k_x_*error_x_;
    omega = omega_ref_ + k_y_*error_y_ + k_yaw_*error_yaw_;

    // Ensure that the linear velocity does not exceed the maximum allowed
    if (vel > vel_max_) {
      vel = vel_max_;
    }
  
    // Ensure that the linear velocity is not negative
    if (vel < 0.0) {
      vel = 0;
    }

    // Ensure that the angular velocity does not exceed the maximum allowed
    if (fabs(omega) > omega_max_) {
      omega = copysign(omega_max_, omega);
    }

    cmd_vel.linear.x = vel;
    cmd_vel.angular.z = omega;

    // ROS_INFO("Velocity : %f", vel);

    // Publish velocity command
    // cmd_vel_pub_.publish(cmd_vel);

    ros::spinOnce();
    rate.sleep();
  }
}

void ControllerConstTrajectory::computeReferenceStates(double time, double freq,
                                                      double x_offset, double y_offset,
                                                      double A) {
  x_ref_ = x_offset + A*sin(freq*time);
  y_ref_ = y_offset + A*sin(2*freq*time);
  dx_ref_ = freq*A*cos(freq*time);
  dy_ref_ = 2*freq*A*cos(2*freq*time);
  ddx_ref_ = -freq*freq*A*sin(freq*time);
  ddy_ref_ = -4*freq*freq*A*sin(2*freq*time);
}

void ControllerConstTrajectory::makeReferenceTrajectory(double freq, double x_offset, double y_offset, double A) {
  // Just making a reference path to Rviz
  int N = 150;

  double Ts = 0.5;  

  // Just two rows
  MatrixXd m(2, N);
  for (int i = 0; i < N; ++i) {
    computeReferenceStates(i*Ts, freq, x_offset, y_offset, A);
    m(0, i) = x_ref_;
    m(1, i) = y_ref_;
  }
  ref_states_matrix_ = m;
}

} // namespace trajectory_tracking_control
