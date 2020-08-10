// Copyright

// TODO Move to test folder
// TODO make controller.h methods more encapsulated

#include <ros/ros.h>

// ROS Messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


// Angles
#include <angles/angles.h>

// Eigen
#include <eigen3/Eigen/Core>

// C++
#include <vector>
#include <string>
#include <cmath>

using Eigen::MatrixXd;

geometry_msgs::Pose robot_pose;
double A = 2.5;
double x_offset = 0.0;
double y_offset = 0.0;
double freq = 2*M_PI/30;

// tf2_ros::Buffer tfBuffer;
// tf2_ros::TransformListener tfListener(tfBuffer);
// geometry_msgs::TransformStamped transformStamped;

// void getPoseFromTF() {
//   try {
//       transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
//   }
//   catch (tf2::TransformException &ex) {
//     ROS_WARN("%s",ex.what());
//     ros::Duration(1.0).sleep();
//   }

//   robot_pose.position.x = transformStamped.transform.translation.x;
//   robot_pose.position.y = transformStamped.transform.translation.y;

//   robot_pose.orientation.x = transformStamped.transform.rotation.x;
//   robot_pose.orientation.y = transformStamped.transform.rotation.y;
//   robot_pose.orientation.z = transformStamped.transform.rotation.z;
//   robot_pose.orientation.w = transformStamped.transform.rotation.w;
// }

void publishReferencePose(double x, double y, double yaw, const ros::Publisher &ref_pose_pub) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;

  quat_tf.setRPY(0, 0, yaw);
  quat_tf.normalize();
  quat_msg = tf2::toMsg(quat_tf);

  // Set orientation
  pose.pose.orientation = quat_msg;
  ref_pose_pub.publish(pose);
}

void makeReferencePath(const ros::Publisher &ref_path_pub) {
  geometry_msgs::PoseArray path;

  // make Reference State Matrix

  int N = round(30.0/0.05);

  for (int t = 0; t < N; ++t) {
    geometry_msgs::Pose pose;
    pose.position.x = x_offset + A*sin(freq*t);
    pose.position.y = y_offset + A*sin(2*freq*t);
    path.poses.push_back(pose);
  }

  ref_path_pub.publish(path);
}

void updateCurrentPose(const nav_msgs::Odometry & msg) {
  robot_pose.position.x = msg.pose.pose.position.x;
  robot_pose.position.y = msg.pose.pose.position.y;

  robot_pose.orientation.x = msg.pose.pose.orientation.x;
  robot_pose.orientation.y = msg.pose.pose.orientation.y;
  robot_pose.orientation.z = msg.pose.pose.orientation.z;
  robot_pose.orientation.w = msg.pose.pose.orientation.w;

}

void updateCurrentPose2(const geometry_msgs::PoseWithCovarianceStamped & msg) {
  // robot_pose.position.x = msg.pose.pose.position.x;
  // robot_pose.position.y = msg.pose.pose.position.y;

  // robot_pose.orientation.x = msg.pose.pose.orientation.x;
  // robot_pose.orientation.y = msg.pose.pose.orientation.y;
  // robot_pose.orientation.z = msg.pose.pose.orientation.z;
  // robot_pose.orientation.w = msg.pose.pose.orientation.w;

  // ROS_INFO("Curr position (%f , %f)", robot_pose.position.x, robot_pose.position.y);
}

void odomCB(const nav_msgs::Odometry & msg) {
  updateCurrentPose(msg);
}

void amclCB(const geometry_msgs::PoseWithCovarianceStamped & msg) {
  // ROS_INFO("Curr position (%f , %f)", msg.pose.pose.orientation.x, msg.pose.pose.orientation.y);
  updateCurrentPose2(msg);
}

double getYawFromQuaternion(const geometry_msgs::Quaternion & quat_msg) {
  double roll, pitch, yaw;

  tf2::Quaternion quat(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  tf2::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "controller_test_ideal_time_scaling");
  ros::NodeHandle nh;
  ros::Rate rate(40);

  // Publishers
  ros::Publisher ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("reference_pose", 100, true);;
  ros::Publisher ref_path_pub = nh.advertise<geometry_msgs::PoseArray>("reference_planner", 100, true);
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);

  // Subscribers
  ros::Subscriber pose_odom_sub = nh.subscribe("/odometry/filtered", 100, &odomCB);
  ros::Subscriber pose_amcl_sub = nh.subscribe("/amcl", 100, &amclCB);

  ros::Duration(2.0).sleep();

  // Reference
  double x_ref, y_ref, dx_ref, dy_ref, ddx_ref, ddy_ref;
  double yaw_ref, yaw_curr;
  double vel_ref, omega_ref;
  double vel, omega;
  double error_x, error_y, error_yaw;
  
  // Control Gains
  double g = 1.5;
  double zeta = 30;
  double k_x, k_y, k_yaw;

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

  makeReferencePath(ref_path_pub);

  double vel_max = 0.5;
  double omega_max = 1.0;

  while (ros::ok()) {


    delta_t = ros::Time::now() - zero_time;
    delta_t_sec = delta_t.toSec();

    // Reference State in time delta_t_sec
    x_ref = x_offset + A*sin(freq*delta_t_sec);
    y_ref = y_offset + A*sin(2*freq*delta_t_sec);
    dx_ref = freq*A*cos(freq*delta_t_sec);
    dy_ref = 2*freq*A*cos(2*freq*delta_t_sec);
    ddx_ref = -freq*freq*A*sin(freq*delta_t_sec);
    ddy_ref = -4*freq*freq*A*sin(2*freq*delta_t_sec);

    yaw_ref = atan2(dy_ref, dx_ref);

    q_ref << x_ref, y_ref, yaw_ref;

    // Publish reference pose
    publishReferencePose(x_ref, y_ref, yaw_ref, ref_pose_pub);

    yaw_curr = getYawFromQuaternion(robot_pose.orientation);

    q_curr << robot_pose.position.x, robot_pose.position.y, yaw_curr;

    // ROS_INFO("ref position (%f , %f)", x_ref, y_ref);
    // ROS_INFO("Curr position (%f , %f)", robot_pose.position.x, robot_pose.position.y);

    vel_ref = sqrt(dx_ref*dx_ref + dy_ref*dy_ref);

    omega_ref = (dx_ref*ddy_ref - dy_ref*ddx_ref)/(dx_ref*dx_ref + dy_ref*dy_ref);

    tf_to_global << cos(yaw_curr), sin(yaw_curr), 0.0,
                    -sin(yaw_curr), cos(yaw_curr), 0.0,
                    0.0, 0.0, 1.0;

    // Posture Error
    error = tf_to_global * (q_ref - q_curr);

    // Wrap to PI yaw error
    error(2, 0) = angles::normalize_angle(error(2, 0));

    // Control
    error_x = error(0, 0);
    error_y = error(1, 0);
    error_yaw = error(2, 0);

    k_x = 2*zeta*sqrt(omega_ref*omega_ref + g*vel_ref*vel_ref);
    k_y = g*vel_ref;
    k_yaw = k_x;

    vel = vel_ref*cos(error_yaw) + k_x*error_x;
    omega = omega_ref + k_y*error_y + k_yaw*error_yaw;

    // Ensure that the linear velocity does not exceed the maximum allowed
    if (vel > vel_max) {
      vel = vel_max;
    }

    if (vel < 0.0) {
      vel = 0.0;
    }

    // Ensure that the angular velocity does not exceed the maximum allowed
    if (fabs(omega) > omega_max) {
      omega = copysign(omega_max, omega);
    }

    cmd_vel.linear.x = vel;
    cmd_vel.angular.z = omega;

    cmd_vel_pub.publish(cmd_vel);

    ros::spinOnce();
    rate.sleep();
  }

}
