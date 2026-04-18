#ifndef DDDMR_DOCKING__TAG_TRACKING_HPP_
#define DDDMR_DOCKING__TAG_TRACKING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace dddmr_docking
{

class TagTracking
{
public:
  TagTracking(rclcpp::Node* node);
  ~TagTracking();

  tf2::Transform getTf2B2Chgpp() const { return tf2_b2chgpp_; }
  tf2::Transform getTf2B2LeftPivot() const { return tf2_b2left_pivot_; }
  tf2::Transform getTf2B2RightPivot() const { return tf2_b2right_pivot_; }
  void startTracking();
  void stopTracking();
  bool isTrackingValid();
  bool odom_received_;
  bool tag_received_;
  bool tf_initialized_;

private:

  rclcpp::Clock::SharedPtr clock_;

  void checkInitTf();
  void onTimer();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void tagPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  rclcpp::Node* node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr tracking_timer_;
  geometry_msgs::msg::PoseStamped camera_to_tag_pose_;
  geometry_msgs::msg::PoseStamped trans_tag2chgpp_;
  geometry_msgs::msg::PoseStamped trans_chgpp2left_pivot_;
  geometry_msgs::msg::PoseStamped trans_chgpp2right_pivot_;
  tf2::Transform tf2_b2chgpp_;
  tf2::Transform tf2_b2c_;
  tf2::Transform tf2_copt2tag_;
  tf2::Transform tf2_tag2chgpp_;
  tf2::Transform tf2_chgpp2left_pivot_;
  tf2::Transform tf2_chgpp2right_pivot_;
  tf2::Transform tf2_b2left_pivot_;
  tf2::Transform tf2_b2right_pivot_;
  tf2::Transform tf2_lastb2tag_;
  tf2::Transform tf2_odom2lastb_;
  tf2::Transform tf2_odom2b_;
  
  std::map<std::string, tf2::Transform> tf2_b2copt_map_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::Odometry latest_tag_odom_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_sub_;
  geometry_msgs::msg::PoseStamped current_tag_pose_;
  bool tf_sent_;

};

}  // namespace dddmr_docking

#endif  // DDDMR_DOCKING__TAG_TRACKING_HPP_
