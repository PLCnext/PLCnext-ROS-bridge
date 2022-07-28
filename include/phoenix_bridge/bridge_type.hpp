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

#ifndef PHOENIX_BRIDGE__BRIDGE_TYPE_HPP_
#define PHOENIX_BRIDGE__BRIDGE_TYPE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "phoenix_bridge/include_types.h"
#include "phoenix_bridge/phoenix_comm.hpp"

/**
 * @brief Class to handle creating a bridge per supported ROS msg type
 * We create one BridgeType for odom, one for twist, etc...
 * A given bridge type can have several "ports" under it, where a port is a publisher or a subscriber
 * This way we can have several pub/subs for odom, several pub/subs for twist etc..
 * The number of ports is derived from config/interface_description.yaml
 */
template <typename T>
class BridgeType : public rclcpp::Node
{
public:
  explicit BridgeType(std::string node_name);
  void init();

private:
  PhoenixComm<T> comm_;  /// Communication layer object. Must be separately initialised.
  std::vector<std::string> pub_topics_, pub_datapaths_;  /// Contain captured parameters
  std::vector<std::string> sub_topics_, sub_datapaths_;  /// Contain captured parameters
  std::vector<int64_t> pub_freqs_, sub_freqs_;           /// Contain captured parameters
  std::vector<rclcpp::TimerBase::SharedPtr>
    pub_timers_;  /// Contain spawned publisher timers to hold them in scope
  std::vector<typename rclcpp::Publisher<T>::SharedPtr>
    pubs_;  /// Contain spawned publishers to hold them in scope
  std::vector<typename rclcpp::Subscription<T>::SharedPtr>
    subs_;  /// Contain spawned subscribers to hold them in scope

  bool getPortParams();
  void spawnPublishers();
  void spawnSubscribers();
};

/**
 * @brief Constructor to create node. Logical init decoupled to allow flexibility.
 *
 * @tparam T Template type parameter
 * @param node_name Name of the node
 */
template <typename T>
inline BridgeType<T>::BridgeType(std::string node_name) : Node(node_name)
{
  init();
}

/**
 * @brief Read the rosparams to create ports and spawn pubs & subs
 */
template <typename T>
inline void BridgeType<T>::init()
{
  if (!this->getPortParams()) return;

  /// Initialise communication layer
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Initialising gRCP channel @" << this->get_parameter("grpc.address").as_string());
  comm_.init(this->get_parameter("grpc.address").as_string());

  /// Spawn pubs
  this->spawnPublishers();

  /// Spawn subs
  this->spawnSubscribers();
}

/**
 * @brief Read params for this node. Param names are hardcoded so the param file should have a very specific structure.
 * @param param_name The fully resolved param path ex: "/odometry/publishers"
 * @param port_vector Save all the read param data into a vector of PortParams
 */
template <typename T>
inline bool BridgeType<T>::getPortParams()
{
  /// @todo Try catch everything
  this->declare_parameter("grpc.address", "unix:/run/plcnext/grpc.sock");  /// Channel address
  this->declare_parameter("grpc.type", "");                                /// Unused
  this->declare_parameter("msg_type", "");  /// The type of the topic to create
  this->declare_parameter(
    "publishers.topics", std::vector<std::string>());  /// Array of publisher topic names
  /// Array of corresponding instance paths in PLC GDS for the varaible to write to
  this->declare_parameter("publishers.datapaths", std::vector<std::string>());
  /// Array of corresponding publisher frequencies
  this->declare_parameter("publishers.frequencies", std::vector<int64_t>());
  this->declare_parameter(
    "subscribers.topics", std::vector<std::string>());  /// Array of subscriber topic names
  /// Array of corresponding instance paths in PLC GDS for the varaible to write to
  this->declare_parameter("subscribers.datapaths", std::vector<std::string>());
  this->declare_parameter("subscribers.frequencies", std::vector<int64_t>());  /// Unused

  pub_topics_ = this->get_parameter("publishers.topics").as_string_array();
  pub_datapaths_ = this->get_parameter("publishers.datapaths").as_string_array();
  pub_freqs_ = this->get_parameter("publishers.frequencies").as_integer_array();

  sub_topics_ = this->get_parameter("subscribers.topics").as_string_array();
  sub_datapaths_ = this->get_parameter("subscribers.datapaths").as_string_array();
  sub_freqs_ = this->get_parameter("subscribers.frequencies").as_integer_array();
  /// The sizes of all 3 publisher arrays should be equal, since the data corresponds per index
  if (!((pub_topics_.size() == pub_datapaths_.size()) &&
        (pub_topics_.size() == pub_freqs_.size()))) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Publisher array sizes not equal!!");
    rclcpp::shutdown();
  }
  /// The sizes of all 3 subscriber arrays should be equal, since the data corresponds per index
  if (!((sub_topics_.size() == sub_datapaths_.size()) &&
        (sub_topics_.size() == sub_freqs_.size()))) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Subscriber array sizes not equal!!");
    rclcpp::shutdown();
  }
  return true;
}

/**
* @brief Spawn publishers based on the read rosparams for this type
*/
template <typename T>
inline void BridgeType<T>::spawnPublishers()
{
  for (size_t i = 0; i < pub_topics_.size(); i++) {
    RCLCPP_INFO(
      this->get_logger(), "Spawning publisher  [%s, %s, %d]", pub_topics_[i].c_str(),
      pub_datapaths_[i].c_str(), pub_freqs_[i]);

    typename rclcpp::Publisher<T>::SharedPtr pub = this->create_publisher<T>(pub_topics_[i], 1000);
    pubs_.push_back(pub);
    pub_timers_.push_back(this->create_wall_timer(
      std::chrono::milliseconds(1000 / pub_freqs_[i]), [this, i, pub]() -> void {
        T msg_rcvd;
        if (comm_.getFromPLC(pub_datapaths_[i], msg_rcvd)) {
          pub->publish(msg_rcvd);
        } else {
          RCLCPP_ERROR_STREAM_ONCE(
            this->get_logger(),
            sub_topics_[i] << " Failed to get data from PLC at " << sub_datapaths_[i]);
        }
      }));
  }
}

/**
 * @brief Spawn subscribers based on the captures rosparams for this type
*/
template <typename T>
inline void BridgeType<T>::spawnSubscribers()
{
  for (size_t i = 0; i < sub_topics_.size(); i++) {
    {
      RCLCPP_INFO(
        this->get_logger(), "Spawning subscriber  [%s, %s, %d]", sub_topics_[i].c_str(),
        sub_datapaths_[i].c_str(), sub_freqs_[i]);
      subs_.push_back(this->create_subscription<T>(
        sub_topics_[i], 10, [this, i](const typename T::SharedPtr msg) -> void {
          if (!comm_.sendToPLC(sub_datapaths_[i], *msg.get()))
            RCLCPP_ERROR_STREAM_ONCE(
              this->get_logger(),
              sub_topics_[i] << " Failed to send data to PLC at " << sub_datapaths_[i]);
        }));
    }
  }
}

#endif  // PHOENIX_BRIDGE__BRIDGE_TYPE_HPP_
