#ifndef DDDMR_DOCKING__TAG_TRACKING_HPP_
#define DDDMR_DOCKING__TAG_TRACKING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dddmr_docking
{

class TagTracking
{
public:
  TagTracking(rclcpp::Node* node);
  ~TagTracking();

private:
  void checkInitTf();
  void onTimer();

  rclcpp::Node* node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped camera_to_tag_pose_;
  geometry_msgs::msg::PoseStamped trans_b2c_;
  geometry_msgs::msg::PoseStamped trans_tag2chgpp_;
  geometry_msgs::msg::PoseStamped trans_chgpp2left_pivot_;
  geometry_msgs::msg::PoseStamped trans_chgpp2right_pivot_;
  tf2::Transform tf2_b2chgpp_;
  tf2::Transform tf2_b2c_;
  tf2::Transform tf2_c2tag_;
  tf2::Transform tf2_tag2chgpp_;
  tf2::Transform tf2_chgpp2left_pivot_;
  tf2::Transform tf2_chgpp2right_pivot_;
  tf2::Transform tf2_b2left_pivot_;
  tf2::Transform tf2_b2right_pivot_;
};

}  // namespace dddmr_docking

#endif  // DDDMR_DOCKING__TAG_TRACKING_HPP_
