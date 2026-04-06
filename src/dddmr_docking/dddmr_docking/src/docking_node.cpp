#include <rclcpp/rclcpp.hpp>
#include "dddmr_docking/mpc_docking.hpp"
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("docking_node");
  
  auto mpc_docking = std::make_unique<dddmr_docking::MPCDocking>(node.get());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
