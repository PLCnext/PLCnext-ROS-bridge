#ifndef INCLUDE_TYPES_H
#define INCLUDE_TYPES_H

/// Include all the ROS message types that should be bridged and the communication layer

#include "phoenix_bridge/dummy_phoenix_comm.h"

#include <ros/ros.h>

/*[[[cog
import cog
import sys
import os

sys.path.append(os.getcwd()) # Necessary when build invokes this script

from src.parsers.param_parser import ParamParser

obj = ParamParser()
for type in obj.types_:
        cog.outl("#include <{}/{}.h>".format(type[0], type[1]))
]]]*/
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//[[[end]]]


#endif // INCLUDE_TYPES_H
