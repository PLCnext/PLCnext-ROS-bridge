#include "phoenix_bridge/phoenix_bridge.hpp"
#include "phoenix_bridge/params_struct.hpp"

#include <ros/ros.h>

/**
 * @brief Create bridges for each type. Parameter passed is the simple name to lookup in config/test_params.yaml
 */
PhoenixBridge::PhoenixBridge()
{
  odom_bridge_.init("odometry");
  twist_bridge_.init("twist");
}
