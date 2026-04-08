#include "dddmr_docking/mpc_docking.hpp"
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking {

MPCDocking::MPCDocking(const std::string &name) : Node(name) {
  tag_tracking_ = std::make_unique<dddmr_docking::TagTracking>(this);
  trajectory_generator_ =
      std::make_unique<dddmr_docking::TrajectoryGenerator>(this);

  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_ = rclcpp_action::create_server<dddmr_sys_core::action::TagDocking>(
    this,
    "/tag_docking",
    std::bind(&MPCDocking::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MPCDocking::handle_cancel, this, std::placeholders::_1),
    std::bind(&MPCDocking::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_
  );

  RCLCPP_INFO(
      this->get_logger(),
      "MPCDocking action server initialized with Tag Tracking and Trajectory Generator.");
}

MPCDocking::~MPCDocking() {}

rclcpp_action::GoalResponse MPCDocking::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::TagDocking::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received goal request for TagDocking");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MPCDocking::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MPCDocking::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> goal_handle)
{
  if (current_handle_ != nullptr && current_handle_->is_active()) {
    RCLCPP_INFO(this->get_logger(), "An older goal is active, cancelling current one.");
    auto result = std::make_shared<dddmr_sys_core::action::TagDocking::Result>();
    result->succeed = false;
    current_handle_->abort(result);
  }
  
  current_handle_ = goal_handle;

  std::thread{std::bind(&MPCDocking::executeCb, this, std::placeholders::_1), goal_handle}.detach();
}

void MPCDocking::executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> goal_handle)
{
  rclcpp::Rate loop_rate(20);
  auto result = std::make_shared<dddmr_sys_core::action::TagDocking::Result>();
  rclcpp::Time success_start_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  RCLCPP_INFO(this->get_logger(), "Executing goal");

  while(rclcpp::ok() && goal_handle->is_active()) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      geometry_msgs::msg::Twist cmd_vel;
      if (cmd_vel_pub_) cmd_vel_pub_->publish(cmd_vel);
      return;
    }

    controlLoop();

    if (tag_tracking_) {
      tf2::Transform t_chgpp = tag_tracking_->getTf2B2Chgpp();
      double cx = t_chgpp.getOrigin().x();
      double cy = t_chgpp.getOrigin().y();
      
      if (std::hypot(cx, cy) < 0.01) {
        if (success_start_time.nanoseconds() == 0) {
          success_start_time = this->now();
        } else if ((this->now() - success_start_time).seconds() >= 3.0) {
          geometry_msgs::msg::Twist cmd_vel;
          if (cmd_vel_pub_) cmd_vel_pub_->publish(cmd_vel);
          result->succeed = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded: tag within 0.01m for 3 seconds.");
          return;
        }
      } else {
        success_start_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      }
    }

    loop_rate.sleep();
  }
}

void MPCDocking::ratingChgPP(dddmr_docking::Trajectory &path) {

  if (path.score_<0 || path.path_.poses.empty() || !tag_tracking_) {
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
  path.score_ += 10.0 / (distance + 0.1); // max score 10

  //RCLCPP_INFO(node_->get_logger(), "distance: %.3f, linear: %.2f, angular: %.2f, score: %.2f",
  //            distance, path.v_, path.w_, path.score_);
}

void MPCDocking::ratingCrossing(dddmr_docking::Trajectory &path) {

  if (path.score_<0 || path.path_.poses.size() < 2 || !tag_tracking_) {
    path.score_ = -1;
    return;
  }

  double px = path.path_.poses.back().pose.position.x;
  double py = path.path_.poses.back().pose.position.y;

  tf2::Transform t_chgpp = tag_tracking_->getTf2B2Chgpp();
  double p1x = t_chgpp.getOrigin().x();
  double p1y = t_chgpp.getOrigin().y();

  tf2::Transform t_left = tag_tracking_->getTf2B2LeftPivot();
  tf2::Transform t_right = tag_tracking_->getTf2B2RightPivot();

  double p2x = (t_left.getOrigin().x() + t_right.getOrigin().x()) / 2.0;
  double p2y = (t_left.getOrigin().y() + t_right.getOrigin().y()) / 2.0;

  double numerator = std::abs((p2x - p1x) * (p1y - py) - (p1x - px) * (p2y - p1y));
  double denominator = std::hypot(p2x - p1x, p2y - p1y);
  double distance = 0.0;
  if (denominator > 1e-6) {
    distance = numerator / denominator;
    path.score_ += 1.0 / (distance + 0.1);
  } else {
    distance = std::hypot(px - p1x, py - p1y);
    path.score_ += 1.0 / (distance + 0.1);
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
    //ratingInTriangle(*it);
    //
    //ratingCrossing(*it);
    ratingCrossing(*it);
    ratingChgPP(*it);;
  }

  if (!trajectory_generator_->generated_trajectories_.empty()) {
    auto best_traj_it = std::max_element(
        trajectory_generator_->generated_trajectories_.begin(),
        trajectory_generator_->generated_trajectories_.end(),
        [](const dddmr_docking::Trajectory &a,
           const dddmr_docking::Trajectory &b) { return a.score_ < b.score_; });

    if (best_traj_it != trajectory_generator_->generated_trajectories_.end()) {
      //RCLCPP_INFO(node_->get_logger(),
      //            "Best trajectory score: %f, v: %f, w: %f",
      //            best_traj_it->score_, best_traj_it->v_, best_traj_it->w_);

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
