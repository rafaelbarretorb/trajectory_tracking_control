/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef TRAJECTORY_TRACKING_CONTROL_CONTROLLER_HPP_
#define TRAJECTORY_TRACKING_CONTROL_CONTROLLER_HPP_


namespace trajectory_tracking_control {

class Controller {
 public:
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) = 0; //NOLINT

  virtual void loadControllerParams() = 0;

  virtual void displayControllerInfo() = 0;

  virtual void updateReferenceState(double time) = 0;

  virtual bool isGoalReached() = 0;

  virtual ~Controller();

//  protected:
//   Controller();
};
}  // namespace 

#endif  // TRAJECTORY_TRACKING_CONTROL_CONTROLLER_HPP_