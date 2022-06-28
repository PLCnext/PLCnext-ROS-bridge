#include <rclcpp/rclcpp.hpp>

#include "phoenix_bridge/bridge_type.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor mte;

  /*[[[cog
  import cog
  import sys
  import os

  sys.path.append(os.getcwd()) # Necessary when colcon build invokes this script

  from phoenix_bridge.param_parser import ParamParser, getResolvedTypeName

  parser = ParamParser()
  for node in parser.nodes_:
        cog.outl("auto {} = std::make_shared<BridgeType<{}>>(\"{}\");"
              .format(node.node_name, getResolvedTypeName(node.header_name), node.node_name))
        cog.outl("mte.add_node({});".format(node.node_name))
        cog.outl()
  ]]]*/
  auto odom_bridge = std::make_shared<BridgeType<nav_msgs::msg::Odometry>>("odom_bridge");
  mte.add_node(odom_bridge);

  auto twist_bridge = std::make_shared<BridgeType<geometry_msgs::msg::Twist>>("twist_bridge");
  mte.add_node(twist_bridge);

  auto string_bridge = std::make_shared<BridgeType<std_msgs::msg::String>>("string_bridge");
  mte.add_node(string_bridge);

  //[[[end]]]

  std::cout << "Node starting" << std::endl;
  mte.spin();
  rclcpp::shutdown();
  return 0;
}
