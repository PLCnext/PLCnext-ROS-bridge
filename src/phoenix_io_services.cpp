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

#include "phoenix_bridge/phoenix_io_services.hpp"

#include <functional>
#include <memory>
#include <string>

using phoenix_bridge::srv::AnalogIO;
using phoenix_bridge::srv::BatchGetIO;
using phoenix_bridge::srv::BatchSetIO;
using phoenix_bridge::srv::SingleGetIO;
using phoenix_bridge::srv::SingleSetIO;

PhoenixIOServices::PhoenixIOServices(
  const std::string node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialising services");

  this->declare_parameter("grpc.address");

  digital_comm_.init(this->get_parameter("grpc.address").as_string());
  analog_comm_.init(this->get_parameter("grpc.address").as_string());

  single_set_service_ = this->create_service<SingleSetIO>(
    "single_set_IO",
    std::bind(&PhoenixIOServices::singleSetCB, this, std::placeholders::_1, std::placeholders::_2));
  single_get_service_ = this->create_service<SingleGetIO>(
    "single_get_IO",
    std::bind(&PhoenixIOServices::singleGetCB, this, std::placeholders::_1, std::placeholders::_2));
  batch_set_service_ = this->create_service<BatchSetIO>(
    "batch_set_IO",
    std::bind(&PhoenixIOServices::batchSetCB, this, std::placeholders::_1, std::placeholders::_2));
  batch_get_service_ = this->create_service<BatchGetIO>(
    "batch_get_IO",
    std::bind(&PhoenixIOServices::batchGetCB, this, std::placeholders::_1, std::placeholders::_2));
  read_analog_service = this->create_service<AnalogIO>(
    "read_analog_IO",
    std::bind(
      &PhoenixIOServices::readAnalogIOCB, this, std::placeholders::_1, std::placeholders::_2));
  write_analog_service = this->create_service<AnalogIO>(
    "write_analog_IO",
    std::bind(
      &PhoenixIOServices::writeAnalogIOCB, this, std::placeholders::_1, std::placeholders::_2));
}

bool PhoenixIOServices::singleSetCB(
  const std::shared_ptr<SingleSetIO::Request> request,
  std::shared_ptr<SingleSetIO::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Single set service called");
  response->status = digital_comm_.sendToPLC(request->datapath, request->value);
  if (!response->status) {
    RCLCPP_WARN_STREAM(this->get_logger(), " set_single_IO failed");
  }
  return response->status;
}

bool PhoenixIOServices::singleGetCB(
  const std::shared_ptr<SingleGetIO::Request> request,
  std::shared_ptr<SingleGetIO::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Single get service called");
  bool val;
  response->status = digital_comm_.getFromPLC(request->datapath, val);
  response->value = val;
  if (!response->status) {
    RCLCPP_WARN_STREAM(this->get_logger(), "single_get_service failed");
  }
  return response->status;
}

bool PhoenixIOServices::batchSetCB(
  const std::shared_ptr<BatchSetIO::Request> request,
  std::shared_ptr<BatchSetIO::Response> response)
{
  if (request->payload.size() == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "batch_set_service got empty payload request!!");
    response->status = false;
    return false;
  }
  response->status = true;
  for (uint64_t i = 0; i < request->payload.size(); i++) {
    response->status =
      digital_comm_.sendToPLC(request->payload[i].datapath, request->payload[i].value);
    if (!response->status) {
      RCLCPP_WARN_STREAM(
        this->get_logger(), "batch_set_service failed setting " << request->payload[i].datapath);
      return false;
    }
  }
  return response->status;
}

bool PhoenixIOServices::batchGetCB(
  const std::shared_ptr<BatchGetIO::Request> request,
  std::shared_ptr<BatchGetIO::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Batch set service called");
  if (request->datapaths.size() == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "batch_get_service got empty payload request!!");
    response->status = false;
    return false;
  }
  response->status = true;

  for (uint64_t i = 0; i < request->datapaths.size(); i++) {
    bool val = true;
    if (!digital_comm_.getFromPLC(request->datapaths[i], val)) {
      response->status = false;
      RCLCPP_WARN_STREAM(
        this->get_logger(), "batch_get_service failed getting " << request->datapaths[i]);
      return false;
    }
    response->values.push_back(val);
  }
  return response->status;
}

bool PhoenixIOServices::readAnalogIOCB(
  const std::shared_ptr<AnalogIO::Request> request, std::shared_ptr<AnalogIO::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Read analog IO service called");
  double val;
  response->status = analog_comm_.getFromPLC(request->instance_path, val);
  response->instance_path = request->instance_path;
  response->value = val;
  if (!response->status) {
    RCLCPP_WARN_STREAM(this->get_logger(), "read_analog_IO failed");
  }
  return response->status;
}

bool PhoenixIOServices::writeAnalogIOCB(
  const std::shared_ptr<AnalogIO::Request> request, std::shared_ptr<AnalogIO::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Write analog IO service called");
  response->status = analog_comm_.sendToPLC(request->instance_path, request->value);
  response->instance_path = request->instance_path;
  if (!response->status) {
    RCLCPP_WARN_STREAM(this->get_logger(), " write_analog_IO failed");
  }
  return response->status;
}
