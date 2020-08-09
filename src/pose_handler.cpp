#include <trajectory_tracking_control/pose_handler.h>


namespace trajectory_tracking_control {
PoseHandler::PoseHandler(tf2_ros::Buffer* tf) : tf_(tf), tfl_(*tf_) {

}

const geometry_msgs::Pose &PoseHandler::getRobotPose() {
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

void PoseHandler::publishReferencePose(double x, double y, double yaw,
                                       const ros::Publisher &pub) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = global_frame_;
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
  pub.publish(pose);
}

void PoseHandler::makeReferencePath() {
  geometry_msgs::PoseArray path;

  for (int i = 0; i < ref_states_matrix_.cols(); ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = ref_states_matrix_(0, i);
    pose.position.y = ref_states_matrix_(1, i);
    path.poses.push_back(pose);
  }
  ref_path_pub_.publish(path);
}
}  // namespace trajectory_tracking_control