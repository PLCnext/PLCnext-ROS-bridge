#include "phoenix_bridge/phoenix_bridge.hpp"
#include "phoenix_bridge/params_struct.hpp"

#include <ros/ros.h>

/**
 * @brief Create bridges for each type. Parameter passed is the simple name to lookup in config/test_params.yaml
 */
PhoenixBridge::PhoenixBridge(ros::NodeHandle nh)
{
  /*[[[cog
  import cog
  import sys
  import os

  sys.path.append(os.getcwd()) # Necessary when build invokes this script
  from src.parsers.param_parser import ParamParser

  obj = ParamParser()
  for type in obj.types_:
          cog.outl(  "if (nh.hasParam(\"{}/{}\"))                   "   .format(type[0], type[1])                                 )
          cog.outl("{")
          cog.outl(  "  ROS_INFO_STREAM(\" Spawing {}/{} bridge \");  " .format(type[0], type[1])                        )
          cog.outl(  "  {}_bridge_.init(\"{}/{}\", nh);            "    .format(type[1].lower(), type[0], type[1])       )
          cog.outl("}")
  ]]]*/
  if (nh.hasParam("nav_msgs/Odometry"))                   
  {
    ROS_INFO_STREAM(" Spawing nav_msgs/Odometry bridge ");  
    odometry_bridge_.init("nav_msgs/Odometry", nh);            
  }
  if (nh.hasParam("geometry_msgs/Twist"))                   
  {
    ROS_INFO_STREAM(" Spawing geometry_msgs/Twist bridge ");  
    twist_bridge_.init("geometry_msgs/Twist", nh);            
  }
  if (nh.hasParam("std_msgs/String"))                   
  {
    ROS_INFO_STREAM(" Spawing std_msgs/String bridge ");  
    string_bridge_.init("std_msgs/String", nh);            
  }
  //[[[end]]]
}
