#include "tag_tracking.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking
{

TagTracking::TagTracking(rclcpp::Node* node)
: node_(node), odom_received_(false)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  init_timer_ = node_->create_wall_timer(
    200ms, std::bind(&TagTracking::checkInitTf, this));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 2,
      std::bind(&TagTracking::odomCallback, this,
                std::placeholders::_1));

  tag_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "tag_pose", 2,
      std::bind(&TagTracking::tagPoseCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "TagTracking initialized. Waiting for base_link to camera TF...");
}

void TagTracking::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
  tf2::fromMsg(current_odom_.pose.pose, tf2_odom2b_);
  odom_received_ = true;
}

void TagTracking::tagPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_tag_pose_ = *msg;
  if(!tf2_b2c_map_.count(msg->header.frame_id)){
    try {
      geometry_msgs::msg::PoseStamped trans_b2c;
      auto transform_b2c = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
      trans_b2c.header = transform_b2c.header;
      trans_b2c.pose.position.x = transform_b2c.transform.translation.x;
      trans_b2c.pose.position.y = transform_b2c.transform.translation.y;
      trans_b2c.pose.position.z = transform_b2c.transform.translation.z;
      trans_b2c.pose.orientation = transform_b2c.transform.rotation;
      tf2::Transform tf2b2c;
      tf2::fromMsg(trans_b2c.pose, tf2b2c);
      tf2_b2c_map_[msg->header.frame_id] = tf2b2c;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to lookup baselink to sensor TF: %s", ex.what());
      return; // Exit and try again next tick
    }
  }
  latest_tag_odom_ = current_odom_;
  tf2::fromMsg(current_tag_pose_.pose, tf2_c2tag_);
  tf2::fromMsg(latest_tag_odom_.pose.pose, tf2_odom2lastb_);
  tf2_lastb2tag_ = tf2_b2c_map_[msg->header.frame_id] * tf2_c2tag_;
}

void TagTracking::checkInitTf()
{
  std::string error_msg;
  if (tf_buffer_->canTransform("base_link", "camera", tf2::TimePointZero, &error_msg) &&
      tf_buffer_->canTransform("tag", "charging_parking_point", tf2::TimePointZero, &error_msg) &&
      tf_buffer_->canTransform("charging_parking_point", "charging_left_pivot_point", tf2::TimePointZero, &error_msg) &&
      tf_buffer_->canTransform("charging_parking_point", "charging_right_pivot_point", tf2::TimePointZero, &error_msg)) {
      
    RCLCPP_INFO(node_->get_logger(), "Received all static TFs. Starting tracking timer.");
    
    try {
      auto transform_tag2cpp = tf_buffer_->lookupTransform("tag", "charging_parking_point", tf2::TimePointZero);
      trans_tag2chgpp_.header = transform_tag2cpp.header;
      trans_tag2chgpp_.pose.position.x = transform_tag2cpp.transform.translation.x;
      trans_tag2chgpp_.pose.position.y = transform_tag2cpp.transform.translation.y;
      trans_tag2chgpp_.pose.position.z = transform_tag2cpp.transform.translation.z;
      trans_tag2chgpp_.pose.orientation = transform_tag2cpp.transform.rotation;
      tf2::fromMsg(trans_tag2chgpp_.pose, tf2_tag2chgpp_);
      
      auto transform_chgpp2left_pivot = tf_buffer_->lookupTransform("charging_parking_point", "charging_left_pivot_point", tf2::TimePointZero);
      trans_chgpp2left_pivot_.header = transform_chgpp2left_pivot.header;
      trans_chgpp2left_pivot_.pose.position.x = transform_chgpp2left_pivot.transform.translation.x;
      trans_chgpp2left_pivot_.pose.position.y = transform_chgpp2left_pivot.transform.translation.y;
      trans_chgpp2left_pivot_.pose.position.z = transform_chgpp2left_pivot.transform.translation.z;
      trans_chgpp2left_pivot_.pose.orientation = transform_chgpp2left_pivot.transform.rotation;
      tf2::fromMsg(trans_chgpp2left_pivot_.pose, tf2_chgpp2left_pivot_);

      auto transform_chgpp2right_pivot = tf_buffer_->lookupTransform("charging_parking_point", "charging_right_pivot_point", tf2::TimePointZero);
      trans_chgpp2right_pivot_.header = transform_chgpp2right_pivot.header;
      trans_chgpp2right_pivot_.pose.position.x = transform_chgpp2right_pivot.transform.translation.x;
      trans_chgpp2right_pivot_.pose.position.y = transform_chgpp2right_pivot.transform.translation.y;
      trans_chgpp2right_pivot_.pose.position.z = transform_chgpp2right_pivot.transform.translation.z;
      trans_chgpp2right_pivot_.pose.orientation = transform_chgpp2right_pivot.transform.rotation;
      tf2::fromMsg(trans_chgpp2right_pivot_.pose, tf2_chgpp2right_pivot_);
      
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to lookup static TF: %s", ex.what());
      return; // Exit and try again next tick
    }

    init_timer_->cancel();
    
    // Switch to 20Hz tracking loop
    timer_ = node_->create_wall_timer(
      50ms, std::bind(&TagTracking::onTimer, this));
  }
}

void TagTracking::onTimer()
{
  
  //pre_integrate_tag_pose;
  //in theory, if odom and tag are the latest, tf2_odom2lastb and tf2_odom2b should be the same
  //when tag is being blocked, the difference between tf2_odom2b and tf2_odom2lastb will be used to compensate tag pose
  
  if(!odom_received_){
    RCLCPP_ERROR(node_->get_logger(), "Check your odom topic!");
    return;
  }
    
  //odom has moved:
  tf2::Transform tf2_b2lastb = tf2_odom2b_.inverse()*tf2_odom2lastb_;
  tf2::Transform tf2_b2tag = tf2_b2lastb*tf2_lastb2tag_;

  tf2_b2chgpp_ = tf2_b2tag * tf2_tag2chgpp_;
  tf2_b2left_pivot_ = tf2_b2chgpp_ * tf2_chgpp2left_pivot_;
  tf2_b2right_pivot_ = tf2_b2chgpp_ * tf2_chgpp2right_pivot_;
  
}

TagTracking::~TagTracking()
{
  // Cleanup
}

}  // namespace dddmr_docking
