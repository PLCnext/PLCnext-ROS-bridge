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

#include "phoenix_bridge/phoenix_io_services.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "phoenix_bridge_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  PhoenixIOServices io(nh);
  ROS_INFO_STREAM(ros::this_node::getName() << " Node started");
  ros::waitForShutdown();
  return 0;
}
