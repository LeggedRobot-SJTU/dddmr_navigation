#include "dddmr_docking/mpc_docking.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking
{

MPCDocking::MPCDocking(rclcpp::Node* node)
: node_(node)
{
  tag_tracking_ = std::make_unique<dddmr_docking::TagTracking>(node_);
  trajectory_generator_ = std::make_unique<dddmr_docking::TrajectoryGenerator>(node_);
  
  control_timer_ = node_->create_wall_timer(
    50ms, std::bind(&MPCDocking::controlLoop, this));

  RCLCPP_INFO(node_->get_logger(), "MPCDocking initialized with Tag Tracking and Trajectory Generator.");
}

MPCDocking::~MPCDocking()
{
}

void MPCDocking::controlLoop()
{
  // 20Hz control logic to be executed here
}

}  // namespace dddmr_docking
