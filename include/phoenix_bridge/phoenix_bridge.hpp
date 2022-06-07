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
  /*[[[cog
  import cog
  import sys
  import os

  sys.path.append(os.getcwd()) # Necessary when build invokes this script
  from src.parsers.param_parser import ParamParser

  obj = ParamParser()
  for type in obj.types_:
          cog.outl( "BridgeType<{}::{}> {}_bridge_;" .format(type[0], type[1], type[1].lower()) )
  ]]]*/
  BridgeType<nav_msgs::Odometry> odometry_bridge_;
  BridgeType<geometry_msgs::Twist> twist_bridge_;
  BridgeType<std_msgs::String> string_bridge_;
  //[[[end]]]
};

#endif // PHOENIX_BRIDGE_HPP
