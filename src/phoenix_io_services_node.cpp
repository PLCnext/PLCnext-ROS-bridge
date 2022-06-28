#include "phoenix_bridge/phoenix_io_services.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor mte;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<PhoenixIOServices>("phoenix_services", options);
  mte.add_node(node);
  mte.spin();
  rclcpp::shutdown();
  return 0;
}
