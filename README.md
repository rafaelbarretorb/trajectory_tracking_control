## ```trajectory_tracking_control```

ROS Trajectory Tracking Control of differential drive using Kinematic Trajectory-Tracking
Error Model

Code based on the algorithms presented in the Chapter 3 - Control of Wheeled Mobile
Systems of the book "Wheeled Mobile Robotics: From Fundamentals Towards Autonomous Systems".

Using B-Spline Interpolation to generate reference trajectories.


Test Node using fix curve

![](docs/husky_controller.gif)

### 1. NODES

#### 1.1 ```trajectory_controller```

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

This node provides an implementation of the ServiceServer that computes the reference states (position, velocity, and acceleration) of a trajectory given a path (set of planar positions), the average velocity desired, and the sampling time. The reference trajectories are generated using **Cubic B-Spline Interpolation**.

##### 1.2.1 Expected Behavior

##### 1.2.2 Action API

##### 1.2.3 Subscribed Topics

##### 1.2.4 Published Topics

##### 1.2.5 Services

##### 1.2.6 Parameters