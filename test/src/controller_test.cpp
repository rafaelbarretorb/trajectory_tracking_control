


#include <ros/ros.h>

// ROS Messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Eigen
#include <eigen3/Eigen/Core>

// C++
#include <vector>
#include <string>
#include <cmath>

using Eigen::MatrixXd;

geometry_msgs::Pose robot_pose;

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

void makeReferencePath(const ros::Publisher &ref_path_pub, const MatrixXd &ref_states_matrix) {
  geometry_msgs::PoseArray path;

  for (int i = 0; i < ref_states_matrix.cols(); ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = ref_states_matrix(0, i);
    pose.position.y = ref_states_matrix(1, i);
    path.poses.push_back(pose);
  }

  ref_path_pub.publish(path);
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "controller_test_ideal_time_scaling");
  ros::NodeHandle nh;
  ros::Rate rate(20);

  // Reference
  double x_ref, y_ref, dx_ref, dy_ref, ddx_ref, ddy_ref;



  // ROS Time
  ros::Time zero_time = ros::Time::now();
  ros::Duration delta_t;
  double delta_t_sec;

  double freq = 2*M_PI/30;


  // Velocity Publisher

  while (ros::ok()) {
    //
    delta_t = ros::Time::now() - zero_time;
    delta_t_sec = delta_t.toSec();

    // Reference State in time delta_t_sec
    x_ref = 1.1 + 0.7*sin(freq*delta_t_sec);
    y_ref = 0.9 + 0.7*sin(2*freq*delta_t_sec);
    dx_ref = freq*0.7*cos(freq*delta_t_sec);
    dy_ref = 2*freq*0.7*cos(2*freq*delta_t_sec);
    ddx_ref = -freq*freq*0.7*sin(freq*delta_t_sec);
    ddy_ref = -4*freq*freq*0.7*sin(2*freq*delta_t_sec);


    rate.sleep();
  }

    return 0;
}

