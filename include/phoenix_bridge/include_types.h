//
// Copyright 2022 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifndef INCLUDE_TYPES_H
#define INCLUDE_TYPES_H

/// Include all the ROS message types that should be bridged and the communication layer

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
