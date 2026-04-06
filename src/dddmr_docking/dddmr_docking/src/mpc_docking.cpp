#include "dddmr_docking/mpc_docking.hpp"
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking {

MPCDocking::MPCDocking(rclcpp::Node *node) : node_(node) {
  tag_tracking_ = std::make_unique<dddmr_docking::TagTracking>(node_);
  trajectory_generator_ =
      std::make_unique<dddmr_docking::TrajectoryGenerator>(node_);

  cmd_vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  control_timer_ =
      node_->create_wall_timer(50ms, std::bind(&MPCDocking::controlLoop, this));

  RCLCPP_INFO(
      node_->get_logger(),
      "MPCDocking initialized with Tag Tracking and Trajectory Generator.");
}

MPCDocking::~MPCDocking() {}

void MPCDocking::ratingInTriangle(dddmr_docking::Trajectory &path) {

  if (path.path_.poses.empty()) {
    path.score_ = -1;
    return;
  }

  // Get the last point of trajectory path
  double px = path.path_.poses.back().pose.position.x;
  double py = path.path_.poses.back().pose.position.y;

  if (!tag_tracking_) {
    path.score_ = -1;
    return;
  }

  tf2::Transform t1 = tag_tracking_->getTf2B2Chgpp();
  double p1x = t1.getOrigin().x();
  double p1y = t1.getOrigin().y();

  tf2::Transform t2 = tag_tracking_->getTf2B2LeftPivot();
  double p2x = t2.getOrigin().x();
  double p2y = t2.getOrigin().y();

  tf2::Transform t3 = tag_tracking_->getTf2B2RightPivot();
  double p3x = t3.getOrigin().x();
  double p3y = t3.getOrigin().y();

  // Point in triangle test (barycentric coordinates proxy / edge side check)
  auto sign = [](double p1x, double p1y, double p2x, double p2y, double p3x,
                 double p3y) {
    return (p1x - p3x) * (p2y - p3y) - (p2x - p3x) * (p1y - p3y);
  };

  bool has_neg = (sign(px, py, p1x, p1y, p2x, p2y) < 0.0) ||
                 (sign(px, py, p2x, p2y, p3x, p3y) < 0.0) ||
                 (sign(px, py, p3x, p3y, p1x, p1y) < 0.0);

  bool has_pos = (sign(px, py, p1x, p1y, p2x, p2y) > 0.0) ||
                 (sign(px, py, p2x, p2y, p3x, p3y) > 0.0) ||
                 (sign(px, py, p3x, p3y, p1x, p1y) > 0.0);

  bool is_inside = !(has_neg && has_pos);

  if (is_inside) {
    // You can update the path rating here e.g.
    // path.score_ += 10.0;
  } else {
    path.score_ = -1;
  }
}

void MPCDocking::ratingChgPP(dddmr_docking::Trajectory &path) {

  if (path.path_.poses.empty() || !tag_tracking_) {
    path.score_ = -1;
    return;
  }

  double px = path.path_.poses.back().pose.position.x;
  double py = path.path_.poses.back().pose.position.y;

  tf2::Transform tf2_b2chgpp = tag_tracking_->getTf2B2Chgpp();
  double cx = tf2_b2chgpp.getOrigin().x();
  double cy = tf2_b2chgpp.getOrigin().y();

  double distance = std::hypot(px - cx, py - cy);

  // Using the distance as a penalty for the score, e.g.:
  path.score_ = 1.0 / (distance + 0.1); // max score 10
}

void MPCDocking::ratingCrossing(dddmr_docking::Trajectory &path) {

  if (path.path_.poses.size() < 2 || !tag_tracking_) {
    path.score_ = -1;
    return;
  }

  double px = path.path_.poses.back().pose.position.x;
  double py = path.path_.poses.back().pose.position.y;

  tf2::Transform t_left = tag_tracking_->getTf2B2LeftPivot();
  double lx = t_left.getOrigin().x();
  double ly = t_left.getOrigin().y();

  tf2::Transform t_right = tag_tracking_->getTf2B2RightPivot();
  double rx = t_right.getOrigin().x();
  double ry = t_right.getOrigin().y();

  double distance_left = std::hypot(px - lx, py - ly);
  double distance_right = std::hypot(px - rx, py - ry);

  RCLCPP_INFO(node_->get_logger(), "distance_left: %.3f, distance_right: %.3f",
              distance_left, distance_right);

  if (distance_left > distance_right) {
    if (path.w_ < 0)
      path.score_ *= 1.1 * fabs(distance_left - distance_right);
  } else if (distance_left < distance_right) {
    if (path.w_ > 0)
      path.score_ *= 1.1 * fabs(distance_left - distance_right);
  }
}

void MPCDocking::controlLoop() {
  trajectory_generator_->generateTrajectories();

  // TODO:
  // 1 odom frame, when camera disappear, use odom
  // 2 PID in score
  // 3 backward motion
  //  Higher score better
  for (auto it = trajectory_generator_->generated_trajectories_.begin();
       it != trajectory_generator_->generated_trajectories_.end(); it++) {
    ratingInTriangle(*it);
    ratingChgPP(*it);
    ratingCrossing(*it);
  }

  if (!trajectory_generator_->generated_trajectories_.empty()) {
    auto best_traj_it = std::max_element(
        trajectory_generator_->generated_trajectories_.begin(),
        trajectory_generator_->generated_trajectories_.end(),
        [](const dddmr_docking::Trajectory &a,
           const dddmr_docking::Trajectory &b) { return a.score_ < b.score_; });

    if (best_traj_it != trajectory_generator_->generated_trajectories_.end()) {
      RCLCPP_INFO(node_->get_logger(),
                  "Best trajectory score: %f, v: %f, w: %f",
                  best_traj_it->score_, best_traj_it->v_, best_traj_it->w_);

      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = best_traj_it->v_;
      cmd_vel.angular.z = best_traj_it->w_;
      if (cmd_vel_pub_) {
        cmd_vel_pub_->publish(cmd_vel);
      }
    }
  }
}

} // namespace dddmr_docking
