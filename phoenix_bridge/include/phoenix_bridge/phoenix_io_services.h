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

#ifndef PHOENIX_BRIDGE_PHOENIX_IO_SERVICES_H
#define PHOENIX_BRIDGE_PHOENIX_IO_SERVICES_H

#include "phoenix_bridge/SetIO.h"
#include "phoenix_bridge/AnalogIO.h"
#include "phoenix_bridge/BatchGetIO.h"
#include "phoenix_bridge/BatchSetIO.h"
#include "phoenix_bridge/SingleSetIO.h"
#include "phoenix_bridge/SingleGetIO.h"
#include "phoenix_bridge/phoenix_comm.h"

#include <ros/ros.h>

class PhoenixIOServices
{
public:
  explicit PhoenixIOServices(ros::NodeHandle nh);

private:
  ros::ServiceServer batch_set_server;
  ros::ServiceServer batch_get_server;
  ros::ServiceServer single_set_server;
  ros::ServiceServer single_get_server;
  ros::ServiceServer analog_read_server;
  ros::ServiceServer analog_write_server;
  PhoenixComm<bool> comm_;
  PhoenixComm<double> analog_comm_;
  ros::NodeHandle nh_;

  bool batchSetCB(phoenix_bridge::BatchSetIO::Request &req,  // NOLINT(runtime/references)
                    phoenix_bridge::BatchSetIO::Response &res);  // NOLINT(runtime/references)
  bool batchGetCB(phoenix_bridge::BatchGetIO::Request &req,  // NOLINT(runtime/references)
                    phoenix_bridge::BatchGetIO::Response &res);  // NOLINT(runtime/references)
  bool singleSetCB(phoenix_bridge::SingleSetIO::Request &req,  // NOLINT(runtime/references)
                    phoenix_bridge::SingleSetIO::Response &res);  // NOLINT(runtime/references)
  bool singleGetCB(phoenix_bridge::SingleGetIO::Request &req,  // NOLINT(runtime/references)
                    phoenix_bridge::SingleGetIO::Response &res);  // NOLINT(runtime/references)
  bool analogReadCB(phoenix_bridge::AnalogIO::Request &req,  // NOLINT(runtime/references)
                  phoenix_bridge::AnalogIO::Response &res);  // NOLINT(runtime/references)
  bool analogWriteCB(phoenix_bridge::AnalogIO::Request &req,  // NOLINT(runtime/references)
                  phoenix_bridge::AnalogIO::Response &res);  // NOLINT(runtime/references)
};

#endif  // PHOENIX_BRIDGE_PHOENIX_IO_SERVICES_H
