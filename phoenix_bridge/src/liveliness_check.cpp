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

#include <memory>
#include <std_msgs/msg/bool.hpp>
#include <string>

#include "phoenix_bridge/phoenix_comm.hpp"

class LivelinessCheck : public rclcpp::Node
{
public:
  explicit LivelinessCheck(rclcpp::NodeOptions options);

private:
  PhoenixComm<bool> comm_;
  std::string grpc_address_;
  std::string liveliness_bool_;
  double liveliness_timeout_;
  int poll_rate_;
  bool beat_;
  bool previous_beat_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;

  void checkLiveliness();
};

using namespace std::chrono_literals;
LivelinessCheck::LivelinessCheck(rclcpp::NodeOptions options)
: rclcpp::Node("liveliness_check", options)
{
  this->declare_parameter("grpc.address", "unix:/run/plcnext/grpc.sock");
  this->declare_parameter("liveliness_bool", "Arp.Plc.Eclr/xLiveliness");
  this->declare_parameter("liveliness_timeout_s", 10.0);
  this->declare_parameter("poll_rate_hz", 10);

  grpc_address_ = this->get_parameter("grpc.address").as_string();
  liveliness_bool_ = this->get_parameter("liveliness_bool").as_string();
  liveliness_timeout_ = this->get_parameter("liveliness_timeout_s").as_double();
  poll_rate_ = this->get_parameter("poll_rate_hz").as_int();

  beat_ = false;
  previous_beat_ = false;
  comm_.init(grpc_address_);
  pub_status_ = this->create_publisher<std_msgs::msg::Bool>("liveliness_status", 1000);

  timer_ = this->create_wall_timer(
    1000ms / poll_rate_, std::bind(&LivelinessCheck::checkLiveliness, this));

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Init @" << grpc_address_ << " for " << liveliness_bool_ << " with "
                                 << liveliness_timeout_ << "s timout & " << poll_rate_ << " Hz.");
}

void LivelinessCheck::checkLiveliness()
{
  // Set the bool to true from ROS
  if (!comm_.sendToPLC(liveliness_bool_, true)) {
    RCLCPP_WARN_STREAM_ONCE(this->get_logger(), ": Ping to PLC failed!");
  }

  std::this_thread::sleep_for(1s * liveliness_timeout_);

  bool response = true;
  if (!comm_.getFromPLC(liveliness_bool_, response)) {
    RCLCPP_WARN_STREAM_ONCE(this->get_logger(), ": Ping from PLC failed!");
  }

  // Expect response to be set false by PLC
  if (response) {
    RCLCPP_WARN_STREAM(this->get_logger(), ": Failed liveliness check!!");
  }
  std_msgs::msg::Bool msg;
  msg.data = !response;
  pub_status_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor mte;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<LivelinessCheck>(options);
  mte.add_node(node);
  mte.spin();
  rclcpp::shutdown();
  return 0;
}
