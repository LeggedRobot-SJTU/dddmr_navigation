#ifndef DDDMR_DOCKING__MPC_DOCKING_HPP_
#define DDDMR_DOCKING__MPC_DOCKING_HPP_

#include <rclcpp/rclcpp.hpp>
#include "dddmr_docking/tag_tracking.hpp"
#include "dddmr_docking/trajectory_generator.hpp"
#include <memory>

namespace dddmr_docking
{

class MPCDocking
{
public:
  MPCDocking(rclcpp::Node* node);
  ~MPCDocking();

private:
  void controlLoop();

  rclcpp::Node* node_;
  std::unique_ptr<dddmr_docking::TagTracking> tag_tracking_;
  std::unique_ptr<dddmr_docking::TrajectoryGenerator> trajectory_generator_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace dddmr_docking

#endif  // DDDMR_DOCKING__MPC_DOCKING_HPP_
