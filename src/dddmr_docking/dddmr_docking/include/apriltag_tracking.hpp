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
#include <apriltag_lib/cit_common_functions.h>

//cv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace dddmr_docking {

class AprilTagTracking
{
public:
  AprilTagTracking(rclcpp::Node* node, std::string name, bool record_tag);
  ~AprilTagTracking();
  void startDetection();
  void stopDetection();
  
private:

  std::string name_;
  rclcpp::Node* node_;
  rclcpp::Clock::SharedPtr clock_;
  
  //@ AprilTag 2 code's attributes
  std::shared_ptr<apriltag_ros::TagDetector> tag_detector_;
  std::string tag_family_;
  int threads_;
  double decimate_;
  double blur_;
  bool refine_edges_;
  double decode_sharpening_;
  bool debug_;
  int max_hamming_distance_ = 2;  // Tunable, but really, 2 is a good choice. Values of >=3
                                  // consume prohibitively large amounts of memory, and otherwise
                                  // you want the largest value possible.
  std::unordered_map<int, std::string> tag_frames;
  std::unordered_map<int, double> tag_sizes;

  std::string topic_image_raw_;
  std::string topic_image_info_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void detectingLoop();
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::TimerBase::SharedPtr detect_tag_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub_;

  sensor_msgs::msg::Image::ConstSharedPtr msg_img_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr msg_ci_;
  bool img_info_get_;
  double detect_tag_frequency_;
  
  double last_roll_, current_roll_;
  double last_pitch_, current_pitch_;
  double last_yaw_, current_yaw_;
  double is_initial_;

  cv_bridge::CvImagePtr cv_image_;
  std::map<int, double> id_size_map_;
  bool record_tags_;
};

} // namespace dddmr_docking

#endif // DDDMR_DOCKING__APRILTAG_TRACKING_HPP_
