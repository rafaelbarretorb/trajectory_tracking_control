
// Copyright

#include <ros/ros.h>
#include <Eigen/Core>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_eigen");
  ros::NodeHandle nh;

  Eigen::Matrix2f A;
  A << 2, -1, -1, 3;

  ros::spin();
  return 0;
}
