#ifndef BRIDGE_TYPE_H
#define BRIDGE_TYPE_H

#include "phoenix_bridge/include_types.h"
#include "phoenix_bridge/params_struct.hpp"

#include <vector>

/**
 * @brief Class to handle creating a bridge per supported ROS msg type
 * We create one BridgeType for odom, one for twist, etc...
 * A given bridge type can have several "ports" under it, where a port is a publisher or a subscriber
 * This way we can have several pub/subs for odom, several pub/subs for twist etc..
 * The number of ports is derived from config/test_params.yaml
 */
template <typename T>
class BridgeType
{
public:
  BridgeType() {}
  void getPortParams(const std::string param_name, std::vector<PortParams> &port_vector);
  void init(const std::string param_name);

private:
  std::vector<PortParams> pub_params_; /// Parameters of publishers to create
  std::vector<PortParams> sub_params_; /// Parameters of subscribers to create
};

/**
 * @brief Read the ports to create for the given type as defined by rosparams, typically uploaded from config/test_params.yaml
 *        Expect to read a list of lists where the embedded list corresponds
 *          to data fields as defineed by phoenix_contact/params_struct.h
 * @param param_name The fully resolved param path ex: "/odometry/publishers"
 * @param port_vector Save all the read param data into a vector of PortParams
 */
template<typename T> inline
void BridgeType<T>::getPortParams(const std::string param_name, std::vector<PortParams> &port_vector)
{
  /// @todo Verify types as per https://answers.ros.org/question/318544/retrieve-list-of-lists-from-yaml-file-parameter-server/
  /// @todo Try catch everything
  XmlRpc::XmlRpcValue param_list;
  if(!ros::param::get(param_name, param_list))
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << " Could not find " << param_name);
  }
  for(int i=0; i<param_list.size(); i++)
  {
    port_vector.push_back(PortParams(param_list[i][0], param_list[i][1], param_list[i][2]));
  }
  for (auto ele:port_vector)
  {
    ROS_INFO_STREAM(param_name << ": " << ele.name_ << " " << ele.datapath_ << " " << ele.frequency_);
  }

}

/**
 * @brief Read the params to create ports, spawn pubs and subs
 * @param param_name The simple name for the parameters of this type ex: "odometry" or "twist"
 */
template<typename T> inline
void BridgeType<T>::init(const std::string param_name)
{
  getPortParams("/"+ param_name +"/publishers", pub_params_);
  getPortParams("/"+ param_name +"/subscribers", sub_params_);
}

#endif // BRIDGE_TYPE_H
