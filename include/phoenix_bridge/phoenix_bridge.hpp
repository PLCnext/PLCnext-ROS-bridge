#ifndef PHOENIX_BRIDGE_HPP
#define PHOENIX_BRIDGE_HPP

#include "phoenix_bridge/bridge_type.hpp"

#include <vector>
#include <string>

class PhoenixBridge
{
public:
  PhoenixBridge();

private:
  BridgeType<nav_msgs::Odometry> odom_bridge_;
//  BridgeType<geometry_msgs::Twist> twist_bridge_;

};

#endif // PHOENIX_BRIDGE_HPP
