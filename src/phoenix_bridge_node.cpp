#include "phoenix_bridge/phoenix_bridge.hpp"
#include "phoenix_bridge/bridge_type.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor mte;
//  std::shared_ptr<PhoenixBridge> node_ptr = std::make_shared<PhoenixBridge>("bridge");

  auto string_bridge = std::make_shared<BridgeType<std_msgs::msg::String>>("string_bridge");
  mte.add_node(string_bridge);

  auto twist_bridge = std::make_shared<BridgeType<geometry_msgs::msg::Twist>>("twist_bridge");
  mte.add_node(twist_bridge);

  std::cout << "Node starting" << std::endl;
  mte.spin();
  rclcpp::shutdown();
  return 0;
}
