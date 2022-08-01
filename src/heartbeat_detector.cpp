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

class HeartbeatDetector : public rclcpp::Node
{
public:
  explicit HeartbeatDetector(rclcpp::NodeOptions options);

private:
  PhoenixComm<bool> comm_;
  std::string grpc_address_;
  std::string heartbeat_bool_;
  int heartbeat_timeout_;
  int poll_rate_;
  bool beat_;
  bool previous_beat_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;

  void watchdogTimer();
};

using namespace std::chrono_literals;
HeartbeatDetector::HeartbeatDetector(rclcpp::NodeOptions options)
: rclcpp::Node("heartbeat_detector", options)
{
  this->declare_parameter("grpc.address", "unix:/run/plcnext/grpc.sock");
  this->declare_parameter("heartbeat_bool", "Arp.Plc.Eclr/MainInstance.xHeartbeat");
  this->declare_parameter("heartbeat_timeout_s", 60);
  this->declare_parameter("poll_rate_hz", 10);

  grpc_address_ = this->get_parameter("grpc.address").as_string();
  heartbeat_bool_ = this->get_parameter("heartbeat_bool").as_string();
  heartbeat_timeout_ = this->get_parameter("heartbeat_timeout_s").as_int();
  poll_rate_ = this->get_parameter("poll_rate_hz").as_int();

  beat_ = false;
  previous_beat_ = false;
  comm_.init(grpc_address_);
  pub_status_ = this->create_publisher<std_msgs::msg::Bool>("heartbeat_status", 1000);
  timer_ = this->create_wall_timer(1ms, std::bind(&HeartbeatDetector::watchdogTimer, this));

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Init @" << grpc_address_ << " for " << heartbeat_bool_ << " with "
                                 << heartbeat_timeout_ << "s.");
}

void HeartbeatDetector::watchdogTimer()
{
  rclcpp::Time start = this->now();
  rclcpp::Duration timeout(heartbeat_timeout_);
  rclcpp::Rate loop_rate(poll_rate_);

  while (rclcpp::ok()) {
    loop_rate.sleep();
    // If the watchdog times out,
    // then heartbeat check fails and reports status bad
    if (this->now() - start > timeout) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Heartbeat check failed!!");
      pub_status_->publish(false);
      return;
    }

    comm_.getFromPLC(heartbeat_bool_, beat_);
    // If a heartbeat is detected,
    // then the watchdog timer reports status good and resets itself
    if (beat_ != previous_beat_) {
      previous_beat_ = beat_;
      pub_status_->publish(true);
      return;
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor mte;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<HeartbeatDetector>(options);
  mte.add_node(node);
  mte.spin();
  rclcpp::shutdown();
  return 0;
}
