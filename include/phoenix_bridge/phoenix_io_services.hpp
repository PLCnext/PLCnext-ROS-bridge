#ifndef PHOENIXIOSERVICES_HPP
#define PHOENIXIOSERVICES_HPP

#include "phoenix_bridge/SetIO.h"
#include "phoenix_bridge/BatchGetIO.h"
#include "phoenix_bridge/BatchSetIO.h"
#include "phoenix_bridge/SingleSetIO.h"
#include "phoenix_bridge/SingleGetIO.h"
#include "phoenix_bridge/phoenix_comm.hpp"

#include <ros/ros.h>

class PhoenixIOServices
{
public:
  PhoenixIOServices(ros::NodeHandle nh);

private:
  ros::ServiceServer batch_set_server;
  ros::ServiceServer batch_get_server;
  ros::ServiceServer single_set_server;
  ros::ServiceServer single_get_server;
  PhoenixComm<bool> comm_;
  ros::NodeHandle nh_;

  bool batchSetCB(phoenix_bridge::BatchSetIO::Request &req, phoenix_bridge::BatchSetIO::Response &res);
  bool batchGetCB(phoenix_bridge::BatchGetIO::Request &req, phoenix_bridge::BatchGetIO::Response &res);
  bool singleSetCB(phoenix_bridge::SingleSetIO::Request &req, phoenix_bridge::SingleSetIO::Response &res);
  bool singleGetCB(phoenix_bridge::SingleGetIO::Request &req, phoenix_bridge::SingleGetIO::Response &res);
};

#endif // PHOENIXIOSERVICES_HPP
