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

#include "phoenix_bridge/include_types.h"
#include "phoenix_bridge/phoenix_comm.h"

#include <std_msgs/Bool.h>
#include <thread>

class LivelinessCheck
{
public:
  explicit LivelinessCheck(ros::NodeHandle nh);
  void checkLiveliness();

private:
  PhoenixComm<bool> comm_;
  std::string grpc_address_;
  std::string liveliness_bool_;
  float liveliness_timeout_;
  int poll_rate_;
  ros::Publisher pub_status_;
  ros::NodeHandle nh_;
};

LivelinessCheck::LivelinessCheck(ros::NodeHandle nh)
{
  nh_ = nh;

  nh_.param<std::string>("communication/grpc/address", grpc_address_, "unix:/run/plcnext/grpc.sock");
  nh_.param<std::string>("liveliness_bool", liveliness_bool_, "Arp.Plc.Eclr/xLiveliness");
  nh_.param<float>("liveliness_timeout_s", liveliness_timeout_, 10);
  nh_.param<int>("poll_rate_hz", poll_rate_, 10);

  comm_.init(grpc_address_);
  pub_status_ = nh_.advertise<std_msgs::Bool>("liveliness_status", 1000);

  ROS_INFO_STREAM(ros::this_node::getName() << ": Init @" << grpc_address_ << " for " << liveliness_bool_ << " with "
                  << liveliness_timeout_ << "s timout and " << poll_rate_ << " Hz poll rate");
}

void LivelinessCheck::checkLiveliness()
{
  ros::Rate loop_rate(poll_rate_);
  while (ros::ok())
  {
    // Set the bool to true from ROS
    if (!comm_.sendToPLC(liveliness_bool_, true))
    {
      ROS_WARN_STREAM_ONCE(ros::this_node::getName() << ": Ping to PLC failed!");
    }
    ros::Duration(liveliness_timeout_).sleep();

    bool response = true;
    if (!comm_.getFromPLC(liveliness_bool_, response))
    {
      ROS_WARN_STREAM_ONCE(ros::this_node::getName() << ": Ping from PLC failed!");
    }

    // Expect response to be set false by PLC
    if (response)
    {
      ROS_WARN_STREAM(ros::this_node::getName() << ": Failed liveliness check!!");
    }
    std_msgs::Bool msg;
    msg.data = !response;
    pub_status_.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "liveliness_detector");

  ros::NodeHandle nh;
  LivelinessCheck check(nh);
  check.checkLiveliness();

  return 0;
}