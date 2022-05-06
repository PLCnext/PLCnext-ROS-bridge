#include "phoenix_bridge/phoenix_io_services.hpp"

#include <functional>

PhoenixIOServices::PhoenixIOServices(ros::NodeHandle nh):
  nh_(nh)
{
  /// Initialise comm layer
  std::string address;
  if(!nh_.getParam("grpc/address", address))
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << " Could not find grpc/address param");
    return;
  }
  comm_.init(address);

  /// Spawn services
  batch_set_server = nh_.advertiseService("batch_set_io", &PhoenixIOServices::batchSetCB, this);
  batch_get_server = nh_.advertiseService("batch_get_io", &PhoenixIOServices::batchGetCB, this);
  single_set_server = nh_.advertiseService("single_set_io", &PhoenixIOServices::singleSetCB, this);
  single_get_server = nh_.advertiseService("single_get_io", &PhoenixIOServices::singleGetCB, this);

}

bool PhoenixIOServices::batchSetCB(phoenix_bridge::BatchSetIO::Request &req, phoenix_bridge::BatchSetIO::Response &res)
{
  if (req.payload.size() == 0)
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": batch_set_service got empty payload request!!");
    res.status = false;
    return false;
  }
  res.status = true;
  for (int i=0; i< req.payload.size(); i++)
  {
    if(!comm_.sendToPLC(req.payload[i].datapath, req.payload[i].value))
    {
      res.status = false;
      ROS_WARN_STREAM(ros::this_node::getName() << ": batch_set_service failed setting " << req.payload[i].datapath);
    }
  }
  return res.status;
}

bool PhoenixIOServices::batchGetCB(phoenix_bridge::BatchGetIO::Request &req, phoenix_bridge::BatchGetIO::Response &res)
{
  if (req.datapaths.size() == 0)
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": batch_get_service got empty payload request!!");
    res.status = false;
    return false;
  }
  res.status = true;
  for (int i=0; i< req.datapaths.size(); i++)
  {
    bool val;
    if(!comm_.getFromPLC(req.datapaths[i], val))
    {
      res.status = false;
      ROS_WARN_STREAM(ros::this_node::getName() << ": batch_get_service failed getting " << req.datapaths[i]);
    }
    res.values[i] = val;
  }
  return res.status;
}

bool PhoenixIOServices::singleSetCB(phoenix_bridge::SingleSetIO::Request &req, phoenix_bridge::SingleSetIO::Response &res)
{
  res.status = comm_.sendToPLC(req.datapath, req.value);
  if (!res.status)
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": single_set_service failed setting " << req.datapath);
  }
  return res.status;
}

bool PhoenixIOServices::singleGetCB(phoenix_bridge::SingleGetIO::Request &req, phoenix_bridge::SingleGetIO::Response &res)
{
  bool val;
  res.status = comm_.getFromPLC(req.datapath, val);
  res.value = val;
  if (!res.status)
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": single_get_service failed getting " << req.datapath);
  }
  return res.status;
}
