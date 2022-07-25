#include "phoenix_bridge/phoenix_io_services.h"

#include <functional>
#include <string>

PhoenixIOServices::PhoenixIOServices(ros::NodeHandle nh):
  nh_(nh)
{
  /// Initialise comm layer
  std::string address;
  if (!ros::param::get("communication/grpc/address", address))
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << " Could not find grpc address param");
  }

  /// Initialise communication layer
  comm_.init(address);
  analog_comm_.init(address);

  /// Spawn services
  batch_set_server = nh_.advertiseService("batch_set_io", &PhoenixIOServices::batchSetCB, this);
  batch_get_server = nh_.advertiseService("batch_get_io", &PhoenixIOServices::batchGetCB, this);
  single_set_server = nh_.advertiseService("single_set_io", &PhoenixIOServices::singleSetCB, this);
  single_get_server = nh_.advertiseService("single_get_io", &PhoenixIOServices::singleGetCB, this);
  analog_read_server = nh_.advertiseService("read_analog_io", &PhoenixIOServices::analogReadCB, this);
  analog_write_server = nh_.advertiseService("write_analog_io", &PhoenixIOServices::analogWriteCB, this);
}

bool PhoenixIOServices::batchSetCB(phoenix_bridge::BatchSetIO::Request &req,  // NOLINT(runtime/references)
                                    phoenix_bridge::BatchSetIO::Response &res)  // NOLINT(runtime/references)
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
    if (!comm_.sendToPLC(req.payload[i].datapath, req.payload[i].value))
    {
      res.status = false;
      ROS_WARN_STREAM(ros::this_node::getName() << ": batch_set_service failed setting " << req.payload[i].datapath);
    }
  }
  return res.status;
}

bool PhoenixIOServices::batchGetCB(phoenix_bridge::BatchGetIO::Request &req,  // NOLINT(runtime/references)
                                    phoenix_bridge::BatchGetIO::Response &res)  // NOLINT(runtime/references)
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
    if (!comm_.getFromPLC(req.datapaths[i], val))
    {
      res.status = false;
      ROS_WARN_STREAM(ros::this_node::getName() << ": batch_get_service failed getting " << req.datapaths[i]);
    }
    res.values.push_back(val);
  }
  return res.status;
}

bool PhoenixIOServices::singleSetCB(phoenix_bridge::SingleSetIO::Request &req,  // NOLINT(runtime/references)
                                      phoenix_bridge::SingleSetIO::Response &res)  // NOLINT(runtime/references)
{
  res.status = comm_.sendToPLC(req.datapath, req.value);
  if (!res.status)
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": single_set_service failed setting " << req.datapath);
  }
  return res.status;
}

bool PhoenixIOServices::singleGetCB(phoenix_bridge::SingleGetIO::Request &req,  // NOLINT(runtime/references)
                                      phoenix_bridge::SingleGetIO::Response &res)  // NOLINT(runtime/references)
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

bool PhoenixIOServices::analogReadCB(phoenix_bridge::AnalogIO::Request &req,  // NOLINT(runtime/references)
                                      phoenix_bridge::AnalogIO::Response &res)  // NOLINT(runtime/references)
{
  double val;
  res.status = analog_comm_.getFromPLC(req.instance_path, val);
  res.value = val;
  if (!res.status)
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": read_analog_IO failed getting " << req.instance_path);
  }
  return res.status;
}

bool PhoenixIOServices::analogWriteCB(phoenix_bridge::AnalogIO::Request &req,  // NOLINT(runtime/references)
                                      phoenix_bridge::AnalogIO::Response &res)  // NOLINT(runtime/references)
{
  res.status = analog_comm_.sendToPLC(req.instance_path, req.value);
  if (!res.status)
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": write_analog_IO failed writing " << req.instance_path);
  }
  return res.status;
}
