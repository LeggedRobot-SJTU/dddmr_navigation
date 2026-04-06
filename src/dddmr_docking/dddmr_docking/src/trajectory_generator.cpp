#include "dddmr_docking/trajectory_generator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dddmr_docking
{

TrajectoryGenerator::TrajectoryGenerator(rclcpp::Node* node)
: node_(node)
{
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&TrajectoryGenerator::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "TrajectoryGenerator initialized.");
}

void TrajectoryGenerator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = *msg;
}

nav_msgs::msg::Path TrajectoryGenerator::generateTrajectory(double v, double w, double sim_time, double sim_granularity)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = "base_link";

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  
  if (sim_granularity <= 0.0) {
    sim_granularity = 0.1;
  }

  for (double t = 0.0; t <= sim_time; t += sim_granularity) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(pose);

    x += v * cos(theta) * sim_granularity;
    y += v * sin(theta) * sim_granularity;
    theta += w * sim_granularity;
  }

  return path;
}

std::vector<nav_msgs::msg::Path> TrajectoryGenerator::generateTrajectories()
{
  std::vector<nav_msgs::msg::Path> all_trajectories;
  double sim_time = 1.0;
  double sim_granularity = 0.1;
  for (double v = -0.5; v <= 0.5001; v += 0.1) {
    for (double w = -1.0; w <= 1.0001; w += 0.1) {
      nav_msgs::msg::Path p = generateTrajectory(v, w, sim_time, sim_granularity);
      all_trajectories.push_back(p);
    }
  }

  generated_trajectories_ = all_trajectories;
  return all_trajectories;
}

TrajectoryGenerator::~TrajectoryGenerator()
{
  // Cleanup
}

}  // namespace dddmr_docking
