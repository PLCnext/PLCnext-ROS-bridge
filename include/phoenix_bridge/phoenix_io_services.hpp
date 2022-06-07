#ifndef PHOENIX_IO_SERVICES_HPP
#define PHOENIX_IO_SERVICES_HPP

#include <rclcpp/rclcpp.hpp>
#include "phoenix_bridge/dummy_phoenix_comm.h"
#include "phoenix_bridge/msg/set_io.hpp"
#include "phoenix_bridge/srv/single_get_io.hpp"
#include "phoenix_bridge/srv/single_set_io.hpp"
#include "phoenix_bridge/srv/batch_get_io.hpp"
#include "phoenix_bridge/srv/batch_set_io.hpp"
#include "phoenix_bridge/srv/analog_io.hpp"

using namespace phoenix_bridge::srv;

class PhoenixIOServices : public rclcpp::Node
{
public:
  PhoenixIOServices(const std::string node_name, const rclcpp::NodeOptions & options);

private:
  rclcpp::Service<SingleSetIO>::SharedPtr single_set_service_;
  rclcpp::Service<SingleGetIO>::SharedPtr single_get_service_;
  rclcpp::Service<BatchSetIO>::SharedPtr batch_set_service_;
  rclcpp::Service<BatchGetIO>::SharedPtr batch_get_service_;
  rclcpp::Service<AnalogIO>::SharedPtr read_analog_service;
  rclcpp::Service<AnalogIO>::SharedPtr write_analog_service;

  DummyPhoenixComm<bool> digital_comm_;
  DummyPhoenixComm<double> analog_comm_;

  bool singleSetCB(const std::shared_ptr<SingleSetIO::Request> request,
                   std::shared_ptr<SingleSetIO::Response> response);
  bool singleGetCB(const std::shared_ptr<SingleGetIO::Request> request,
                   std::shared_ptr<SingleGetIO::Response> response);
  bool batchSetCB(const std::shared_ptr<BatchSetIO::Request> request,
                   std::shared_ptr<BatchSetIO::Response> response);
  bool batchGetCB(const std::shared_ptr<BatchGetIO::Request> request,
                   std::shared_ptr<BatchGetIO::Response> response);
  bool readAnalogIOCB(const std::shared_ptr<AnalogIO::Request> request,
                      std::shared_ptr<AnalogIO::Response> response);
  bool writeAnalogIOCB(const std::shared_ptr<AnalogIO::Request> request,
                      std::shared_ptr<AnalogIO::Response> response);
};

#endif // PHOENIX_IO_SERVICES_HPP
