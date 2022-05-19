#include "phoenix_bridge/phoenix_io_services.hpp"
#include <functional>

using namespace phoenix_bridge::srv;

PhoenixIOServices::PhoenixIOServices(const std::string node_name,
                                     const rclcpp::NodeOptions &options):
  rclcpp::Node(node_name, options)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialising services");

  comm_.init(this->get_parameter("grpc.address").as_string());

  single_set_service_ =
      this->create_service<SingleSetIO>("single_set_IO",
                                        std::bind(&PhoenixIOServices::singleSetCB,
                                                  this,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2));
  single_get_service_ =
      this->create_service<SingleGetIO>("single_get_IO",
                                        std::bind(&PhoenixIOServices::singleGetCB,
                                                  this,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2));
  batch_set_service_ =
      this->create_service<BatchSetIO>("batch_set_IO",
                                        std::bind(&PhoenixIOServices::batchSetCB,
                                                  this,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2));
  batch_get_service_ =
      this->create_service<BatchGetIO>("batch_get_IO",
                                        std::bind(&PhoenixIOServices::batchGetCB,
                                                  this,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2));
}

bool PhoenixIOServices::singleSetCB(const std::shared_ptr<SingleSetIO::Request> request,
                                    std::shared_ptr<SingleSetIO::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Single set service called");
  response->status = comm_.sendToPLC(request->datapath, request->value);
  if (!response->status)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), " set_single_IO failed");
  }
  return response->status;
}

bool PhoenixIOServices::singleGetCB(const std::shared_ptr<SingleGetIO::Request> request,
                                    std::shared_ptr<SingleGetIO::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Single set service called");
  bool val;
  response->status = comm_.getFromPLC(request->datapath, val);
  response->value = val;
  if (!response->status)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "single_get_service failed");
  }
  return response->status;
}

bool PhoenixIOServices::batchSetCB(const std::shared_ptr<BatchSetIO::Request> request,
                                   std::shared_ptr<BatchSetIO::Response> response)
{
  if (request->payload.size() == 0)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), ": batch_set_service got empty payload request!!");
    response->status = false;
    return false;
  }
  response->status = true;
  for (int i=0; i< request->payload.size(); i++)
  {
    response->status=
        comm_.sendToPLC(request->payload[i].datapath, request->payload[i].value);
    if(!response->status)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), ": batch_set_service failed setting " << request->payload[i].datapath);
      return false;
    }
  }
  return response->status;
}

bool PhoenixIOServices::batchGetCB(const std::shared_ptr<BatchGetIO::Request> request,
                                   std::shared_ptr<BatchGetIO::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Batch set service called");
  if (request->datapaths.size() == 0)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), ": batch_get_service got empty payload request!!");
    response->status = false;
    return false;
  }
  response->status = true;

  for (int i=0; i< request->datapaths.size(); i++)
  {
    bool val = true;
    if(!comm_.getFromPLC(request->datapaths[i], val))
    {
      response->status = false;
      RCLCPP_WARN_STREAM(this->get_logger(), ": batch_get_service failed getting " << request->datapaths[i]);
      return false;
    }
    response->values.push_back(val);
  }
  return response->status;
}
