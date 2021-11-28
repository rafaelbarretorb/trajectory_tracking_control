/*
  Copyright 2021 - Rafael Barreto
*/


#include <geometry_msgs/Twist.h>

#include "trajectory_tracking_control/controller.hpp"

namespace trajectory_tracking_control {

class LyapunovControl : public Controller {
 public:
  LyapunovControl();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
};

}  // namespace trajectory_tracking_control
