#include "trajectory_generator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dddmr_docking {

TrajectoryGenerator::TrajectoryGenerator(rclcpp::Node *node) : node_(node) {

  RCLCPP_INFO(node_->get_logger(), "TrajectoryGenerator initialized.");
}

Trajectory TrajectoryGenerator::generateTrajectory(double v, double w,
                                                   double sim_time,
                                                   double sim_granularity) {
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

  Trajectory trajectory;
  trajectory.path_ = path;
  trajectory.v_ = v;
  trajectory.w_ = w;
  trajectory.score_ = 0.0;
  return trajectory;
}

void TrajectoryGenerator::generateTrajectories(double sim_time) {
  std::vector<Trajectory> all_trajectories;
  double sim_granularity = 0.1;
  for (double v = -0.1; v <= 0.1001; v += 0.05) {
    for (double w = -0.1; w <= 0.11; w += 0.05) {
      if (v == 0)
        continue;
      Trajectory p = generateTrajectory(v, w, sim_time, sim_granularity);
      all_trajectories.push_back(p);
    }
  }

  generated_trajectories_ = all_trajectories;
}

TrajectoryGenerator::~TrajectoryGenerator() {
  // Cleanup
}

} // namespace dddmr_docking
