#ifndef DDDMR_DOCKING__TRAJECTORY_GENERATOR_HPP_
#define DDDMR_DOCKING__TRAJECTORY_GENERATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace dddmr_docking
{

class TrajectoryGenerator
{
public:
  TrajectoryGenerator(rclcpp::Node* node);
  ~TrajectoryGenerator();

  nav_msgs::msg::Path generateTrajectory(double v, double w, double sim_time, double sim_granularity);
  std::vector<nav_msgs::msg::Path> generateTrajectories();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node* node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry current_odom_;
  std::vector<nav_msgs::msg::Path> generated_trajectories_;
};

}  // namespace dddmr_docking

#endif  // DDDMR_DOCKING__TRAJECTORY_GENERATOR_HPP_
