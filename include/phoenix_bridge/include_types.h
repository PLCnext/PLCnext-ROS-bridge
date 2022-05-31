#ifndef INCLUDE_TYPES_H
#define INCLUDE_TYPES_H

/// Include all the ROS message types that should be bridged and the communication layer

#include "phoenix_bridge/dummy_phoenix_comm.h"

#include <rclcpp/rclcpp.hpp>

/*[[[cog
import cog
import sys
import os

sys.path.append(os.getcwd()) # Necessary when colcon build invokes this script

from phoenix_bridge.param_parser import ParamParser, getResolvedTypeName

parser = ParamParser()
for node in parser.nodes_:
      cog.outl("#include<{}.hpp>".format(node.header_name))
]]]*/
#include<nav_msgs/msg/odometry.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<std_msgs/msg/string.hpp>
//[[[end]]]

#endif // INCLUDE_TYPES_H
