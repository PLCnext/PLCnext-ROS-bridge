#ifndef PHOENIX_BRIDGE_PHOENIX_IO_SERVICES_H
#define PHOENIX_BRIDGE_PHOENIX_IO_SERVICES_H

#include "phoenix_bridge/SetIO.h"
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
  PhoenixComm<bool> comm_;
  ros::NodeHandle nh_;

  bool batchSetCB(phoenix_bridge::BatchSetIO::Request &req,  // NOLINT(runtime/references)
                    phoenix_bridge::BatchSetIO::Response &res);  // NOLINT(runtime/references)
  bool batchGetCB(phoenix_bridge::BatchGetIO::Request &req,  // NOLINT(runtime/references)
                    phoenix_bridge::BatchGetIO::Response &res);  // NOLINT(runtime/references)
  bool singleSetCB(phoenix_bridge::SingleSetIO::Request &req,  // NOLINT(runtime/references)
                    phoenix_bridge::SingleSetIO::Response &res);  // NOLINT(runtime/references)
  bool singleGetCB(phoenix_bridge::SingleGetIO::Request &req,  // NOLINT(runtime/references)
                    phoenix_bridge::SingleGetIO::Response &res);  // NOLINT(runtime/references)
};

#endif  // PHOENIX_BRIDGE_PHOENIX_IO_SERVICES_H
