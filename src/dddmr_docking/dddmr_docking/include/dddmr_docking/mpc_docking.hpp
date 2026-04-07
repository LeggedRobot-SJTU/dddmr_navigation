#ifndef DDDMR_DOCKING__MPC_DOCKING_HPP_
#define DDDMR_DOCKING__MPC_DOCKING_HPP_

#include "dddmr_docking/tag_tracking.hpp"
#include "dddmr_docking/trajectory_generator.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace dddmr_docking {

class MPCDocking {
public:
  MPCDocking(rclcpp::Node *node);
  ~MPCDocking();

private:
  void controlLoop();

  rclcpp::Node *node_;
  std::unique_ptr<dddmr_docking::TagTracking> tag_tracking_;
  std::unique_ptr<dddmr_docking::TrajectoryGenerator> trajectory_generator_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  bool ratingIsInTriangle(dddmr_docking::Trajectory &path);
  void ratingInTriangle(dddmr_docking::Trajectory &path);
  void ratingChgPP(dddmr_docking::Trajectory &path);
  void ratingCrossingNull(dddmr_docking::Trajectory &path);
  void ratingCrossing(dddmr_docking::Trajectory &path);
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

} // namespace dddmr_docking

#endif // DDDMR_DOCKING__MPC_DOCKING_HPP_
