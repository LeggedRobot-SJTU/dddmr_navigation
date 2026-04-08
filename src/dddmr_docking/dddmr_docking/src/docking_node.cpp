#include <rclcpp/rclcpp.hpp>
#include "dddmr_docking/mpc_docking.hpp"
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto mpc_docking = std::make_shared<dddmr_docking::MPCDocking>("docking_node");

  rclcpp::spin(mpc_docking);
  rclcpp::shutdown();
  return 0;
}
