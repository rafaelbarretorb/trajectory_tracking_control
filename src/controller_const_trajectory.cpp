#include <trajectory_tracking_control/controller_const_trajectory.h>

namespace trajectory_tracking_control {

ControllerConstTrajectory::ControllerConstTrajectory(ros::NodeHandle *nodehandle,
                                                      tf2_ros::Buffer& tf) : 
                                                      nh_(*nodehandle),
                                                      pose_handler_(&tf) {

  // Initialize Publishers
  ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference_pose", 100, true);;
  ref_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("reference_planner", 100, true);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);

  // Initialize Matrices
  q_ref_ = MatrixXd(3, 1);
  q_curr_ = MatrixXd(3, 1);
  tf_to_global_ = MatrixXd(3, 3);
  error_ = MatrixXd(3, 1);

  // Just the loop function to test the localization for now
  execute();
}

void ControllerConstTrajectory::execute() {
  ros::Rate rate(10);

  ros::Duration(2.0).sleep();
  

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

  pose_handler_.publishReferencePath(ref_path_pub);

  double omega;
  double vel;

  // Max linear and angular velocities
  double vel_max = 0.5;
  double omega_max = 1.0;
  
  // Current pose of the robot
  geometry_msgs::Pose curr_pose;

  double A = 2.5;
  double x_offset = 0.0;
  double y_offset = 0.0;
  double freq = 2*M_PI/30;

  VectorXf ref_states(6);

  while (ros::ok()) {
    // Get time in secs
    delta_t = ros::Time::now() - zero_time;
    delta_t_sec = delta_t.toSec();

    computeReferenceStates(&ref_states, delta_t_sec, freq, x_offset, y_offset, A);
  
    // Update robot pose
    curr_pose = pose_handler_.getRobotPose();

    yaw_ref_ = atan2(dy_ref_, dx_ref_);

    q_ref_ << x_ref_, y_ref_, yaw_ref_;

    // Publish reference pose
    pose_handler_.publishReferencePose(x_ref_, y_ref_, yaw_ref_, ref_pose_pub_);

    yaw_curr_ = pose_handler_.getYawFromQuaternion(curr_pose.orientation);

    q_curr_ << curr_pose.position.x, curr_pose.position.y, yaw_curr_;

    vel_ref_ = sqrt(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);

    omega_ref_ = (dx_ref_*ddy_ref_ - dy_ref_*ddx_ref_)/(dx_ref_*dx_ref_ + dy_ref_*dy_ref_);

  if (vel_ref_ < 0.0) {
    vel_ref_ = 0.0;
  }

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

  if (vel < 0.0){
    vel = 0.0;
  }

    cmd_vel.linear.x = vel;
    cmd_vel.angular.z = omega;

    // Publish velocity command
    // cmd_vel_pub_.publish(cmd_vel);

    ros::spinOnce();
    rate.sleep();
  }
}

void ControllerConstTrajectory::computeReferenceStates(VectorXf *v, double time, double freq,
                                                      double x_offset, double y_offset,
                                                      double A) {
  double x_ref = x_offset + A*sin(freq*time);
  double y_ref = y_offset + A*sin(2*freq*time);
  double dx_ref = freq*A*cos(freq*time);
  double dy_ref = 2*freq*A*cos(2*freq*time);
  double ddx_ref = -freq*freq*A*sin(freq*time);
  double ddy_ref = -4*freq*freq*A*sin(2*freq*time);

  *v << x_ref, y_ref, dx_ref, dy_ref, ddx_ref, ddy_ref;
}

void ControllerConstTrajectory::makeReferenceTrajectory(double freq) {

  // TODO Not finish yet
  double time = 0.0;
  while (time*freq < 2*M_PI) {
    time = time + 0.5;
  }

}

} // namespace trajectory_tracking_control
