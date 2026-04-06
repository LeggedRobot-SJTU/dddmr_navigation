#include "dddmr_docking/tag_tracking.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking
{

TagTracking::TagTracking(rclcpp::Node* node)
: node_(node)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  init_timer_ = node_->create_wall_timer(
    200ms, std::bind(&TagTracking::checkInitTf, this));

  RCLCPP_INFO(node_->get_logger(), "TagTracking initialized. Waiting for base_link to camera TF...");
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
      auto transform_b2c = tf_buffer_->lookupTransform("base_link", "camera", tf2::TimePointZero);
      trans_b2c_.header = transform_b2c.header;
      trans_b2c_.pose.position.x = transform_b2c.transform.translation.x;
      trans_b2c_.pose.position.y = transform_b2c.transform.translation.y;
      trans_b2c_.pose.position.z = transform_b2c.transform.translation.z;
      trans_b2c_.pose.orientation = transform_b2c.transform.rotation;
      tf2::fromMsg(trans_b2c_.pose, tf2_b2c_);

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
  try {
    auto transform = tf_buffer_->lookupTransform("camera", "tag", tf2::TimePointZero);
    camera_to_tag_pose_.header = transform.header;
    camera_to_tag_pose_.pose.position.x = transform.transform.translation.x;
    camera_to_tag_pose_.pose.position.y = transform.transform.translation.y;
    camera_to_tag_pose_.pose.position.z = transform.transform.translation.z;
    camera_to_tag_pose_.pose.orientation = transform.transform.rotation;
    tf2::fromMsg(camera_to_tag_pose_.pose, tf2_c2tag_);
    tf2_b2chgpp_ = tf2_b2c_ * tf2_c2tag_ * tf2_tag2chgpp_;
    tf2_b2left_pivot_ = tf2_b2chgpp_ * tf2_chgpp2left_pivot_;
    tf2_b2right_pivot_ = tf2_b2chgpp_ * tf2_chgpp2right_pivot_;
    
    RCLCPP_INFO(node_->get_logger(), "Calculated base_link -> left_pivot: x=%.3f, y=%.3f", tf2_b2left_pivot_.getOrigin().x(), tf2_b2left_pivot_.getOrigin().y());
    RCLCPP_INFO(node_->get_logger(), "Calculated base_link -> right_pivot: x=%.3f, y=%.3f", tf2_b2right_pivot_.getOrigin().x(), tf2_b2right_pivot_.getOrigin().y());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(node_->get_logger(), "Could not transform from camera to tag: %s", ex.what());
  }
}

TagTracking::~TagTracking()
{
  // Cleanup
}

}  // namespace dddmr_docking
