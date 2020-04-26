
// Copyright Rafael Barreto

#ifndef TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_ // NOLINT
#define TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_

#include <ros/ros.h>

// Messages
#include <geometry_msgs/Point.h>

#include <vector>

namespace trajectory_tracking_control {

class Controller {
 public:
  Controller();
  void makeReferenceTrajectory();

 private:
  std::vector<geometry_msgs::Point> reference_trajectory_;
};
};  // namespace trajectory_tracking_control

#endif  // TRAJECTORY_TRACKING_CONTROL_CONTROLLER_H_ NOLINT
