#ifndef INCLUDE_TYPES_H
#define INCLUDE_TYPES_H

/// Include all the ROS message types that should be bridged and the communication layer

#include "phoenix_bridge/dummy_phoenix_comm.h"

#include <rclcpp/rclcpp.hpp>

/*[[[cog
import cog
from phoenix_bridge.param_parser import ParamParser
obj = ParamParser()
for type in obj.types_:
        cog.outl("#include <{}/msg/{}.h>".format(type[0], type[1]))
]]]*/
//[[[end]]]


#endif // INCLUDE_TYPES_H
