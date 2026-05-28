#include "apriltag_labelling.hpp"
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking {

AprilTagLabelling::AprilTagLabelling(const std::string &name) : Node(name) {

  clock_ = this->get_clock();

  //@Start to load cameras
  this->declare_parameter("cameras", rclcpp::PARAMETER_STRING_ARRAY);
  this->get_parameter("cameras", cameras_);
  for(auto i=cameras_.begin(); i!=cameras_.end(); i++){
    RCLCPP_INFO(this->get_logger(), "Use camera: %s", (*i).c_str());
    bool record_tags = true;
    apriltag_tracking_map_[(*i)] = std::make_shared<dddmr_docking::AprilTagTracking>(this, (*i), record_tags);
  }

  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_ = rclcpp_action::create_server<dddmr_sys_core::action::RecordApriltag>(
    this,
    "tag_docking",
    std::bind(&AprilTagLabelling::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AprilTagLabelling::handle_cancel, this, std::placeholders::_1),
    std::bind(&AprilTagLabelling::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_
  );

  RCLCPP_INFO(
      this->get_logger(),
      "AprilTagLabelling action server initialized with Tag Tracking and Trajectory Generator.");
}

AprilTagLabelling::~AprilTagLabelling() {}

rclcpp_action::GoalResponse AprilTagLabelling::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::RecordApriltag::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received goal request for RecordApriltag");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AprilTagLabelling::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecordApriltag>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AprilTagLabelling::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecordApriltag>> goal_handle)
{
  if (current_handle_ != nullptr && current_handle_->is_active()) {
    RCLCPP_INFO(this->get_logger(), "An older goal is active, cancelling current one.");
    auto result = std::make_shared<dddmr_sys_core::action::RecordApriltag::Result>();
    result->succeed = false;
    current_handle_->abort(result);
  }
  
  current_handle_ = goal_handle;

  std::thread{std::bind(&AprilTagLabelling::executeCb, this, std::placeholders::_1), goal_handle}.detach();
}

void AprilTagLabelling::executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecordApriltag>> goal_handle)
{
  rclcpp::Rate loop_rate(20);
  auto result = std::make_shared<dddmr_sys_core::action::RecordApriltag::Result>();
  
  //@ Activate Tag Detector
  for(auto i=apriltag_tracking_map_.begin();i!=apriltag_tracking_map_.end();i++){
    i->second->startDetection();
  }

  while(rclcpp::ok() && goal_handle->is_active()) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      //Stop Detector
      for(auto i=apriltag_tracking_map_.begin();i!=apriltag_tracking_map_.end();i++){
        i->second->stopDetection();
      }
      return;
    }
    loop_rate.sleep();
  }
}

} // namespace dddmr_docking
