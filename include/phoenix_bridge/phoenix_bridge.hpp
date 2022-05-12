#ifndef PHOENIX_BRIDGE_HPP
#define PHOENIX_BRIDGE_HPP

#include "phoenix_bridge/include_types.h"

#include <vector>
#include <string>

/**
 * @brief Create bridges for each type and initialise them
 */
class PhoenixBridge : public rclcpp::Node
{
public:
  PhoenixBridge(std::string nodename);

private:
  /// Declare all the bridge types to create
  /*[[[cog
  import cog
  from phoenix_bridge.param_parser import ParamParser
  obj = ParamParser()
  for type in obj.types_:
          cog.outl( "BridgeType<{}::{}> {}_bridge_;" .format(type[0], type[1], type[1].lower()) )
  ]]]*/
  //[[[end]]]
};

#endif // PHOENIX_BRIDGE_HPP
