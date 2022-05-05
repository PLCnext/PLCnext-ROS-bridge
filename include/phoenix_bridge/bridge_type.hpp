#ifndef BRIDGE_TYPE_H
#define BRIDGE_TYPE_H

#include "phoenix_bridge/include_types.h"
#include "phoenix_bridge/params_struct.hpp"

#include <vector>

template <typename T>
class BridgeType
{
public:
  BridgeType() {}
  void getPortParams(const std::string param_name, std::vector<PortParams> &port_vector);
  void init(const std::string param_name);

private:
  std::vector<PortParams> pub_params_;
  std::vector<PortParams> sub_params_;
};

template<typename T> inline
void BridgeType<T>::getPortParams(const std::string param_name, std::vector<PortParams> &port_vector)
{
  // TODO: Verify types as per https://answers.ros.org/question/318544/retrieve-list-of-lists-from-yaml-file-parameter-server/
  // TODO: Try catch everything
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
    std::cout << ele.name_ << " " << ele.datapath_ << " " << ele.frequency_ << std::endl;
  }

}

template<typename T> inline
void BridgeType<T>::init(const std::string param_name)
{
  getPortParams("/"+ param_name +"/publishers", pub_params_);
  getPortParams("/"+ param_name +"/subscribers", sub_params_);
}

#endif // BRIDGE_TYPE_H
