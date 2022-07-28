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

#ifndef PHOENIX_BRIDGE__PHOENIX_IO_SERVICES_HPP_
#define PHOENIX_BRIDGE__PHOENIX_IO_SERVICES_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "phoenix_bridge/msg/set_io.hpp"
#include "phoenix_bridge/phoenix_comm.hpp"
#include "phoenix_bridge/srv/analog_io.hpp"
#include "phoenix_bridge/srv/batch_get_io.hpp"
#include "phoenix_bridge/srv/batch_set_io.hpp"
#include "phoenix_bridge/srv/single_get_io.hpp"
#include "phoenix_bridge/srv/single_set_io.hpp"

using phoenix_bridge::srv::AnalogIO;
using phoenix_bridge::srv::BatchGetIO;
using phoenix_bridge::srv::BatchSetIO;
using phoenix_bridge::srv::SingleGetIO;
using phoenix_bridge::srv::SingleSetIO;

/**
 * @brief Class to provide the ros services for setting/getting IOs on the PLC
 * Services are offered to read/write digital IOs in a batch or individually, and analog IOs only individually.
 *
 */
class PhoenixIOServices : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Phoenix I O Services:: Phoenix I O Services object
   *
   * @param node_name Node name
   * @param options Options
   */
  PhoenixIOServices(const std::string node_name, const rclcpp::NodeOptions & options);

private:
  rclcpp::Service<SingleSetIO>::SharedPtr single_set_service_;  /// Write one DIO
  rclcpp::Service<SingleGetIO>::SharedPtr single_get_service_;  /// Read one DIO
  rclcpp::Service<BatchSetIO>::SharedPtr batch_set_service_;    /// Write a batch of DIOs
  rclcpp::Service<BatchGetIO>::SharedPtr batch_get_service_;    /// Read a batch of DIOs
  rclcpp::Service<AnalogIO>::SharedPtr read_analog_service;     /// Read one AIO
  rclcpp::Service<AnalogIO>::SharedPtr write_analog_service;    /// Write one AIO

  PhoenixComm<bool> digital_comm_;   /// Communication layer object for DIOs
  PhoenixComm<double> analog_comm_;  /// Communication layer object for AIOs

  /**
   * @brief Callback for the Single set DIO service
   *
   * @param request DIO instance path and value to set
   * @param response grpc call status
   * @return true If succesfully set
   * @return false If setting failed
   */
  bool singleSetCB(
    const std::shared_ptr<SingleSetIO::Request> request,
    std::shared_ptr<SingleSetIO::Response> response);

  /**
   * @brief Callback for the single get DIO service
   *
   * @param request DIO instance path to read
   * @param response grpc call status and value retrieved
   * @return true If succesfully retrieved
   * @return false If getting failed
   */
  bool singleGetCB(
    const std::shared_ptr<SingleGetIO::Request> request,
    std::shared_ptr<SingleGetIO::Response> response);

  /**
   * @brief Callback for the Batch set DIO service
   *
   * @param request Array of DIO paths and values to set
   * @param response grpc call status
   * @return true If succesfully set
   * @return false If setting failed
   * @todo Replace looped call of single DIO to a single call with arrays of DIOS
   */
  bool batchSetCB(
    const std::shared_ptr<BatchSetIO::Request> request,
    std::shared_ptr<BatchSetIO::Response> response);

  /**
   * @brief Callback for the Batch get DIO service
   *
   * @param request Array of paths to get
   * @param response grpc call status and arrays of retrieved values
   * @return true If succesfully retrieved
   * @return false If getting failed
   * @todo Replace looped call of single DIO to a single call with arrays of DIOS
 */
  bool batchGetCB(
    const std::shared_ptr<BatchGetIO::Request> request,
    std::shared_ptr<BatchGetIO::Response> response);

  /**
   * @brief Callback for the Single read AIO service
   *
   * @param request Instance path of AIO to get
   * @param response grpc call status and value of AIO
   * @return true If succesfully retrieved
   * @return false If getting failed
   */
  bool readAnalogIOCB(
    const std::shared_ptr<AnalogIO::Request> request, std::shared_ptr<AnalogIO::Response> response);

  /**
   * @brief Callback for the Single write AIO service
   *
   * @param request Instance path & value of AIO to write
   * @param response grpc call status
   * @return true If succesfully written
   * @return false If writing failed
   */
  bool writeAnalogIOCB(
    const std::shared_ptr<AnalogIO::Request> request, std::shared_ptr<AnalogIO::Response> response);
};

#endif  // PHOENIX_BRIDGE__PHOENIX_IO_SERVICES_HPP_
