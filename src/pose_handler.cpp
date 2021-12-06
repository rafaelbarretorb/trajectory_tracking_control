/*
  Copyright 2021 - Rafael Barreto
*/

#include "trajectory_tracking_control/pose_handler.hpp"


namespace trajectory_tracking_control {

// TODO(Rafael) tfl_??

PoseHandler::PoseHandler(ros::NodeHandle *nodehandle, tf2_ros::Buffer* tf) : tf_(tf), nh_(*nodehandle)  {
  // TODO(Rafael) remove
  odom_frame_ = "odom";
  global_frame_ = "map";
  robot_base_frame_ = "base_link";
}

geometry_msgs::Pose PoseHandler::getRobotPose() {
  geometry_msgs::TransformStamped transformStamped;
  if (tf_->canTransform(odom_frame_, global_frame_, ros::Time(0))) {
    transformStamped = tf_->lookupTransform(odom_frame_, robot_base_frame_, ros::Time(0));
  }
  robot_pose_.position.x = transformStamped.transform.translation.x;
  robot_pose_.position.y = transformStamped.transform.translation.y;

  robot_pose_.orientation.x = transformStamped.transform.rotation.x;
  robot_pose_.orientation.y = transformStamped.transform.rotation.y;
  robot_pose_.orientation.z = transformStamped.transform.rotation.z;
  robot_pose_.orientation.w = transformStamped.transform.rotation.w;

  return robot_pose_;
}

double PoseHandler::getYawFromQuaternion(const geometry_msgs::Quaternion & quat_msg) {
  double roll, pitch, yaw;

  tf2::Quaternion quat(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  tf2::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);

  return yaw;
}





}  // namespace trajectory_tracking_control
