#ifndef DDDMR_DOCKING__TRAJECTORY_GENERATOR_HPP_
#define DDDMR_DOCKING__TRAJECTORY_GENERATOR_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dddmr_docking {

struct Trajectory {
  nav_msgs::msg::Path path_;
  double v_;
  double w_;
  double score_;
};

class TrajectoryGenerator {
public:
  TrajectoryGenerator(rclcpp::Node *node);
  ~TrajectoryGenerator();

  Trajectory generateTrajectory(double v, double w, double sim_time,
                                double sim_granularity);
  void generateTrajectories();

  std::vector<Trajectory> generated_trajectories_;

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node *node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry current_odom_;
};

} // namespace dddmr_docking

#endif // DDDMR_DOCKING__TRAJECTORY_GENERATOR_HPP_
