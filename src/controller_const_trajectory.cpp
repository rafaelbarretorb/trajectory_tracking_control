#include <trajectory_tracking_control/controller_const_trajectory.h>

namespace trajectory_tracking_control {

ControllerConstTrajectory::ControllerConstTrajectory(ros::NodeHandle *nodehandle,
                                                      tf2_ros::Buffer& tf) : 
                                                      nh_(*nodehandle),
                                                      pose_handler_(&tf) {
// Just the loop function to test the localization for now
execute();
}

void ControllerConstTrajectory::execute() {
  ros::Rate rate(10);
  
  geometry_msgs::Pose pose;
  while (ros::ok()) {
    pose = pose_handler_.getRobotPose();
    ROS_INFO("current pose = ( %f , %f )", pose.position.x, pose.position.y);
    ros::spinOnce();
    rate.sleep();
  }
}

} // namespace trajectory_tracking_control
