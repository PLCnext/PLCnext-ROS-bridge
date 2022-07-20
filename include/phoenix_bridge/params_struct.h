#ifndef PHOENIX_BRIDGE_PARAMS_STRUCT_H
#define PHOENIX_BRIDGE_PARAMS_STRUCT_H

#include <string>

/**
 * @brief The struct to hold the parameters defined per subscriber/pulbisher per datatype in the config/interface_description.yaml file
 *        Since each datatype can have several pub/subs, we will create a vector of these structs to hold all the data
 *        A "Port" is any one given subscriber or publisher
 *        Each type (such as std_msgs/Bool or geometry_msgs/Twist) can have multiple ports. See config/interface_description.yaml
 */
struct PortParams
{
  std::string name_;  /// Topic name
  std::string datapath_;  /// Path to data objecct corresponding to this port on the PLC
  int frequency_;  /// Frequency for publishing. Not applicable to subscribers (??)
  PortParams(std::string name, std::string datapath, int freq):
    name_(name),
    datapath_(datapath),
    frequency_(freq)
    {}
};

#endif  // PHOENIX_BRIDGE_PARAMS_STRUCT_H
