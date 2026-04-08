#ifndef DDDMR_DOCKING__APRILTAG_TRACKING_HPP_
#define DDDMR_DOCKING__APRILTAG_TRACKING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <memory>
#include <angles/angles.h>
//tf2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// apriltag
#include <apriltag.h>
#include <apriltag_lib/tag_functions.hpp>
#include <apriltag_lib/pose_estimation.hpp>

//cv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace dddmr_docking {

class AprilTagTracking
{
public:
  AprilTagTracking(rclcpp::Node* node, std::string name);
  ~AprilTagTracking();

private:

  std::string name_;
  rclcpp::Node* node_;
  rclcpp::Clock::SharedPtr clock_;
  
  //@ apriltag library
  apriltag_family_t* tf;
  apriltag_detector_t* const td;
  std::string tag_family_;
  std::atomic<int> max_hamming;
  std::atomic<bool> profile;
  std::unordered_map<int, std::string> tag_frames;
  std::unordered_map<int, double> tag_sizes;
  std::function<void(apriltag_family_t*)> tf_destructor;
  pose_estimation_f estimate_pose = nullptr;

  std::string topic_image_raw_;
  std::string topic_image_info_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void detectingLoop();
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::TimerBase::SharedPtr detect_tag_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_pub_;

  sensor_msgs::msg::Image::ConstSharedPtr msg_img_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr msg_ci_;
  bool img_info_get_;
  double detect_tag_frequency_;
  
  double last_roll_, current_roll_;
  double last_pitch_, current_pitch_;
  double last_yaw_, current_yaw_;
  double is_initial_;
};

} // namespace dddmr_docking

#endif // DDDMR_DOCKING__APRILTAG_TRACKING_HPP_
