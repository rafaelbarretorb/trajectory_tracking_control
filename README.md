## ```trajectory_tracking_control```

[![ros-action-ci](https://github.com/rafaelbarretorb/trajectory_tracking_control/actions/workflows/action-ros-ci.yaml/badge.svg)](https://github.com/rafaelbarretorb/trajectory_tracking_control/actions/workflows/action-ros-ci.yaml)

ROS Trajectory Tracking Control package for differential drive and skid-steer drive mobile robots. Using B-Spline Interpolation to generate reference trajectories.

### Constant Trajectory
![](docs/const_traj.gif)

### From a given path
![](docs/no_const_traj.gif)


### References

- Books:
  - "Wheeled Mobile Robotics: From Fundamentals Towards Autonomous Systems".

- Articles:
  - a
### Project plan
#### Design types developped:

:heavy_check_mark: Linear Control

⬜️ Lyapunov-Based Control

⬜️ Model-Based Predictive Control


#### Handle Obstacles:
⬜️ handle unknown obstacles in the trajectory seen by the perception sensors

⬜️ Add dynamic reconfiguration

⬜️ Add others methods to generate trajectories

#### Launch

```
```

### 1. NODES

#### 1.1 ```trajectory_controller```
This node provides an implementation of a trajectory tracking controller through a ROS Action Server. Given the action goal, this node computes the velocity commands to move the robot from an initial point to a goal point.

##### 1.1.1 Expected Behavior

##### 1.1.2 Action API

###### Action Subscribed Topics

###### Action Published Topics

##### 1.1.3 Subscribed Topics

##### 1.1.4 Published Topics

##### 1.1.5 Services

~service1()


##### 1.1.6 Parameters
~param1()

~param2()

#### 1.2 ```reference_states_server```

This node provides an implementation of the ROS Service Server that computes the reference states (position, velocity, and acceleration) of a trajectory given a path, the average velocity desired, and the sampling time. The reference trajectories are generated using **Cubic B-Spline Interpolation**.

##### 1.2.1 Expected Behavior

##### 1.2.2 Action API

##### 1.2.3 Subscribed Topics

##### 1.2.4 Published Topics

##### 1.2.5 Services

##### 1.2.6 Parameters