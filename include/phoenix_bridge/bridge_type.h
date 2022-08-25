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

#ifndef PHOENIX_BRIDGE_BRIDGE_TYPE_H
#define PHOENIX_BRIDGE_BRIDGE_TYPE_H

#include "phoenix_bridge/include_types.h"
#include "phoenix_bridge/params_struct.h"
#include "phoenix_bridge/phoenix_comm.h"

#include <string>
#include <thread>  // NOLINT(build/c++11)
#include <vector>
#include <xmlrpcpp/XmlRpcException.h>

/**
 * @brief Class to handle creating a bridge per supported ROS msg type
 * We create one BridgeType for odom, one for twist, etc...
 * A given bridge type can have several "ports" under it, where a port is a publisher or a subscriber
 * This way we can have several pub/subs for odom, several pub/subs for twist etc..
 * The number of ports is derived from config/interface_description.yaml
 */
template <typename T>
class BridgeType
{
public:
  BridgeType() {}
  bool getPortParams(const std::string param_name, std::vector<PortParams> *port_vector);
  void init(const std::string param_name, ros::NodeHandle nh);

private:
  std::vector<PortParams> pub_params_;  /// Parameters of publishers to create
  std::vector<PortParams> sub_params_;  /// Parameters of subscribers to create
  void publisherFunction(const PortParams port);
  PhoenixComm<T> comm_;
  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> subscribers_;
  std::string bridge_name_;
};

/**
 * @brief Read the params to create ports, spawn pubs and subs
 * @param param_name The simple name for the parameters of this type ex: "odometry" or "twist"
 */
template<typename T> inline
void BridgeType<T>::init(const std::string param_name, ros::NodeHandle nh)
{
  nh_ = nh;
  bridge_name_ =  ros::this_node::getName()  + "/" + param_name;
  bool got_pubs =  getPortParams(param_name + "/publishers", &pub_params_);
  bool got_subs =  getPortParams(param_name + "/subscribers", &sub_params_);
  if (!got_pubs && !got_subs)
  {
    return;
  }
  std::string address;
  if (!ros::param::get("communication/grpc/address", address))
  {
    ROS_ERROR_STREAM(bridge_name_ << " Could not find grpc address param, terminating");
    return;
  }
  ROS_INFO_STREAM(bridge_name_ << ": Initialised gRPC channel @" << address);

  /// Initialise communication layer
  comm_.init(address);

  /// Spawn pubs
  for (auto port : pub_params_)
  {
    std::thread thr(std::bind(&BridgeType::publisherFunction, this, port));
    thr.detach();  // Thread must ensure graceful shutdown of itself since it is detached
  }

  /// Spawn subs
  for (auto port : sub_params_)
  {
    subscribers_.
        push_back(nh_.subscribe<T>(port.name_, 10,
                                    [this, port](const boost::shared_ptr<const T> msg) ->
                                    void
                                    {
                                      if (!comm_.sendToPLC(port.datapath_, *msg.get()))
                                        ROS_ERROR_STREAM_ONCE(bridge_name_ << ": "
                                        << port.name_ << " Failed to send data to PLC at " << port.datapath_);
                                    }
                                    ));  // NOLINT(whitespace/parens)
  }
  ROS_INFO_STREAM(bridge_name_ << " Bridge type spawned");
}

/**
 * @brief Read the ports to create for the given type as defined by rosparams, typically uploaded from config/interface_description.yaml
 *        Expect to read a list of lists where the embedded list corresponds
 *          to data fields as defineed by phoenix_contact/params_struct.h
 * @param param_name The fully resolved param path ex: "/odometry/publishers"
 * @param port_vector Save all the read param data into a vector of PortParams
 */
template<typename T> inline
bool BridgeType<T>::getPortParams(const std::string param_name, std::vector<PortParams> *port_vector)
{
  /// @todo Verify types as per https://answers.ros.org/question/318544/retrieve-list-of-lists-from-yaml-file-parameter-server/
  XmlRpc::XmlRpcValue param_list;
  if (!nh_.getParam(param_name, param_list))
  {
    // If no specified param found, silently return false
    return false;
  }
  for (int i = 0; i < param_list.size(); i++)
  {
    if (param_list[i].size() != 3)
    {
      // If params of incorrect format, silently return false and skip
      return false;
    }
    std::string topic_name;
    std::string instance_path;
    int frequency;
    try
    {
      topic_name = static_cast<std::string>(param_list[i][0]);
      instance_path = static_cast<std::string>(param_list[i][1]);
      frequency = static_cast<int>(param_list[i][2]);
    }
    catch (XmlRpc::XmlRpcException e)
    {
      ROS_ERROR_STREAM(ros::this_node::getName() << ": " 
        << param_name << ": Params of incorrect format, expecting [topic_name: string, instance_path: string, frequency: int]");
      return false;
    }
    if (topic_name == "" | instance_path == "" | frequency == 0)
    {
      // If params are incomplete or empty, silently return false and skip
      return false;
    }
    port_vector->push_back(PortParams(topic_name, instance_path, frequency));
  }
  for (auto ele : *port_vector)
  {
    ROS_INFO_STREAM(ros::this_node::getName() << ": " 
      << param_name << ": [" << ele.name_+"," << " " << ele.datapath_+"," << " " << ele.frequency_ << "]");
  }
  return true;
}

/**
 * @brief Function bound to a thread per publisher per BridgeType. Get data from PLC and publish at parametrised frequency
 * @param port PortParam object, specifying which datapath to query data for, which topic to publish on and at what frequency
 */
template<typename T> inline
void BridgeType<T>::publisherFunction(const PortParams port)
{
  ros::Publisher pub = nh_.advertise<T>(port.name_, 1000);  /// @todo Parametrise the publisher buffer size? Pros/Cons?
  while (ros::ok())
  {
    ros::spinOnce();
    T msg_rcvd;
    if (comm_.getFromPLC(port.datapath_, msg_rcvd))
    {
      pub.publish(boost::make_shared<T>(msg_rcvd));  /// 0 copy transfer
    }
    else
    {
      ROS_ERROR_STREAM_ONCE(bridge_name_ << ": " << port.name_ << " Failed to get data from PLC at " << port.datapath_);
    }
    ros::Rate(port.frequency_).sleep();
  }
}

#endif  // PHOENIX_BRIDGE_BRIDGE_TYPE_H
