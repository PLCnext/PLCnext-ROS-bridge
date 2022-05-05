#ifndef PHOENIX_BRIDGE_HPP
#define PHOENIX_BRIDGE_HPP

#include "phoenix_bridge/bridge_type.hpp"

#include <vector>
#include <string>

/**
 * @brief Create bridges for each type and initialise them
 */
class PhoenixBridge
{
public:
  PhoenixBridge(ros::NodeHandle nh);

private:
  /// Declare all the bridge types to create
  BridgeType<nav_msgs::Odometry> odom_bridge_;
  BridgeType<geometry_msgs::Twist> twist_bridge_;
};

#endif // PHOENIX_BRIDGE_HPP
