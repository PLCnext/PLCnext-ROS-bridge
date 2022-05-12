#include "phoenix_bridge/phoenix_bridge.hpp"
#include "phoenix_bridge/params_struct.hpp"

/**
 * @brief Create bridges for each type. Parameter passed is the simple name to lookup in config/test_params.yaml
 */
PhoenixBridge::PhoenixBridge(std::string nodename)
  : Node(nodename)
{
  /*[[[cog
  import cog
  from phoenix_bridge.param_parser import ParamParser
  obj = ParamParser()
  for type in obj.types_:
          cog.outl(  "if (nh.hasParam(\"{}/{}\"))                   "   .format(type[0], type[1])                                 )
          cog.outl("{")
          cog.outl(  "  ROS_INFO_STREAM(\" Spawing {}/{} bridge \");  " .format(type[0], type[1])                        )
          cog.outl(  "  {}_bridge_.init(\"{}/{}\", nh);            "    .format(type[1].lower(), type[0], type[1])       )
          cog.outl("}")
  ]]]*/
  //[[[end]]]
}
