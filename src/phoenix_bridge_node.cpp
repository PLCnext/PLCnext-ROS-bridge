#include "phoenix_bridge/phoenix_bridge.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "phoenix_bridge_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  PhoenixBridge br;
  ROS_INFO_STREAM(ros::this_node::getName() << " Node started");
  return 0;
}
