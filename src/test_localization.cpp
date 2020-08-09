


#include <ros/ros.h>

// ROS Messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


void getRobotPoseFromTF(tf2_ros::Buffer *tfBuffer,
                        geometry_msgs::TransformStamped *transform_stamped,
                        const std::string &target_frame,
                        const std::string &source_frame) {
  if (tfBuffer->canTransform("odom", "map", ros::Time(0))) {
    *transform_stamped = tfBuffer->lookupTransform(target_frame, source_frame, ros::Time(0));
    ROS_INFO("%s pose: (%f , %f)", source_frame.c_str() ,transform_stamped->transform.translation.x, transform_stamped->transform.translation.y);
  }
}

void odomCB(const nav_msgs::Odometry & msg) {
  ROS_INFO("Pose from Odom: (%f , %f)", msg.pose.pose.position.x, msg.pose.pose.position.y);
}

void amclCB(const geometry_msgs::PoseWithCovarianceStamped & msg) {
  ROS_INFO("Pose from AMCL: (%f , %f)\n", msg.pose.pose.position.x, msg.pose.pose.position.y);
}


int main(int argc, char ** argv) {
  ros::init(argc, argv, "test_localization");
  ros::NodeHandle nh;
  ros::Rate rate(10);


  // Subscribers
  ros::Subscriber pose_odom_sub = nh.subscribe("/odometry/filtered", 100, &odomCB);
  // ros::Subscriber pose_amcl_sub = nh.subscribe("/amcl_pose", 100, &amclCB);

  // TF2
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  while (ros::ok()) {
    // getRobotPoseFromTF(&tfBuffer, &transformStamped, "base_link", "map");
    getRobotPoseFromTF(&tfBuffer, &transformStamped, "odom", "base_link");
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
}
