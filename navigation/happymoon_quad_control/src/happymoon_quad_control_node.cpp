#include "happymoon_control.h"
#include <ros/ros.h>
int main(int argc, char **argv) {
  ros::init(argc, argv, "happymoon_quad_control_node");
  // happymoon control
  happymoon_control::HappyMoonControl happymoon_control_;

  ros::spin();

  return 0;
}