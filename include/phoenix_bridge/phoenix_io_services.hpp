#ifndef PHOENIX_IO_SERVICES_HPP
#define PHOENIX_IO_SERVICES_HPP

#include <rclcpp/rclcpp.hpp>
#include "phoenix_bridge/dummy_phoenix_comm.h"
#include "phoenix_bridge/msg/set_io.hpp"
#include "phoenix_bridge/srv/single_get_io.hpp"
#include "phoenix_bridge/srv/single_set_io.hpp"
#include "phoenix_bridge/srv/batch_get_io.hpp"
#include "phoenix_bridge/srv/batch_set_io.hpp"

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
  DummyPhoenixComm<bool> comm_;

  bool singleSetCB(const std::shared_ptr<SingleSetIO::Request> request,
                   std::shared_ptr<SingleSetIO::Response> response);
  bool singleGetCB(const std::shared_ptr<SingleGetIO::Request> request,
                   std::shared_ptr<SingleGetIO::Response> response);
  bool batchSetCB(const std::shared_ptr<BatchSetIO::Request> request,
                   std::shared_ptr<BatchSetIO::Response> response);
  bool batchGetCB(const std::shared_ptr<BatchGetIO::Request> request,
                   std::shared_ptr<BatchGetIO::Response> response);
};

#endif // PHOENIX_IO_SERVICES_HPP
