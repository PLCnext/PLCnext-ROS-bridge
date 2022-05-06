#include "phoenix_bridge/phoenix_bridge.hpp"
#include "phoenix_bridge/params_struct.hpp"

#include <ros/ros.h>

/**
 * @brief Create bridges for each type. Parameter passed is the simple name to lookup in config/test_params.yaml
 */
PhoenixBridge::PhoenixBridge(ros::NodeHandle nh)
{
  if (nh.hasParam("/odometry"))
  {
    ROS_INFO_STREAM("Spawing odometry bridge");
    odom_bridge_.init("/odometry", nh);
  }

  if (nh.hasParam("/twist"))
  {
    ROS_INFO_STREAM("Spawing twist bridge");
    twist_bridge_.init("/twist", nh);
  }
}
