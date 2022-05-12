#include "phoenix_bridge/phoenix_bridge.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor mte;
  std::shared_ptr<PhoenixBridge> node_ptr = std::make_shared<PhoenixBridge>("bridge");
  mte.add_node(node_ptr);
  std::cout << "Node starting" << std::endl;
  mte.spin();
  rclcpp::shutdown();
  return 0;
}
