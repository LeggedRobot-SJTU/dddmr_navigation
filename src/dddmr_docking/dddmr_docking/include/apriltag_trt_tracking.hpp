#ifndef DDDMR_DOCKING__APRILTAG_TRT_TRACKING_HPP_
#define DDDMR_DOCKING__APRILTAG_TRT_TRACKING_HPP_

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

//cv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>
#ifdef TRT_ENABLED
#include "dddmr_trt/yolov8.h"
#include <opencv2/cudaimgproc.hpp>
#endif

namespace dddmr_docking {

class AprilTagTrtTracking
{
public:
  AprilTagTrtTracking(rclcpp::Node* node, std::string name, bool record_tag);
  ~AprilTagTrtTracking();
  void startDetection();
  void stopDetection();
  
private:

  std::string name_;
  rclcpp::Node* node_;
  rclcpp::Clock::SharedPtr clock_;

  std::string topic_image_raw_;
  std::string topic_image_info_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_img_;
  
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void detectingLoop();
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::TimerBase::SharedPtr detect_tag_timer_;

  sensor_msgs::msg::Image::ConstSharedPtr msg_img_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr msg_ci_;
  bool img_info_get_;
  double detect_tag_frequency_;

  cv_bridge::CvImagePtr cv_image_;
  
  std::string trt_model_path_;
  bool is_trt_engine_exist_;
#ifdef TRT_ENABLED
  std::shared_ptr<YoloV8> yolov8_;
#endif
};

} // namespace dddmr_docking

#endif // DDDMR_DOCKING__APRILTAG_TRACKING_HPP_
