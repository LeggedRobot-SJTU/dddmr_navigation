#include <rclcpp/rclcpp.hpp>
#include "apriltag_labelling.hpp"
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto apriltag_labelling = std::make_shared<dddmr_docking::AprilTagLabelling>("apriltag_labelling_node");

  rclcpp::spin(apriltag_labelling);
  rclcpp::shutdown();
  return 0;
}
