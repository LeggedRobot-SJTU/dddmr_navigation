#include "apriltag_tracking.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking {

AprilTagTracking::AprilTagTracking(rclcpp::Node* node, std::string name) : node_(node), name_(name), 
img_info_get_(false), is_initial_(false){
  
  //-----AprilTag Setup-----
  node_->declare_parameter(name_+".tag_family", rclcpp::ParameterValue("36h11"));
  node_->get_parameter(name_+".tag_family", tag_family_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "tag_family: %s", tag_family_.c_str());

  // get tag names, IDs and sizes
  std::vector<int64_t> ids;
  node_->declare_parameter(name_+".tag.ids", rclcpp::ParameterValue(std::vector<int64_t>{}));
  node_->get_parameter(name_+".tag.ids", ids);

  std::vector<double> sizes;
  node_->declare_parameter(name_+".tag.sizes", rclcpp::ParameterValue(std::vector<double>{}));
  node_->get_parameter(name_+".tag.sizes", sizes);
  
  for(size_t i=0;i<ids.size();i++){
    id_size_map_[ids[i]] = sizes[i];
    RCLCPP_INFO(node_->get_logger().get_child(name_), "ID: %lu size: %.5f", ids[i], sizes[i]);
  }
  // detector parameters in "detector" namespace
  node_->declare_parameter(name_+".detector.threads", rclcpp::ParameterValue(2));
  node_->get_parameter(name_+".detector.threads", threads_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "threads: %d", threads_);
  
  node_->declare_parameter(name_+".detector.decimate", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_+".detector.decimate", decimate_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "decimate: %.2f", decimate_);

  node_->declare_parameter(name_+".detector.blur", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_+".detector.blur", blur_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "blur: %.2f", blur_);
  
  node_->declare_parameter(name_+".detector.refine_edges", rclcpp::ParameterValue(true));
  node_->get_parameter(name_+".detector.refine_edges", refine_edges_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "refine_edges: %d", refine_edges_);

  node_->declare_parameter(name_+".detector.decode_sharpening", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_+".detector.decode_sharpening", decode_sharpening_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "decode_sharpening: %.2f", decode_sharpening_);
  
  node_->declare_parameter(name_+".detector.debug", rclcpp::ParameterValue(false));
  node_->get_parameter(name_+".detector.debug", debug_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "debug: %d", debug_);
  
  node_->declare_parameter(name_+".max_hamming_distance", rclcpp::ParameterValue(2));
  node_->get_parameter(name_+".max_hamming_distance", max_hamming_distance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_hamming_distance: %d", max_hamming_distance_);

//  TagDetector(std::string family, int thread, double decimate, 
//                        double blur, bool refine_edges, double decode_sharpening_, int debug, int max_hamming_distance);
  tag_detector_ = std::make_shared<apriltag_ros::TagDetector>(tag_family_, threads_, decimate_, 
                                              blur_, refine_edges_, decode_sharpening_, debug_, max_hamming_distance_, id_size_map_);
  //-----AprilTag Setup End-----

  //add clock
  clock_ = node_->get_clock();

  node_->declare_parameter(name_+".topic_image_raw", rclcpp::ParameterValue("image_raw"));
  node_->get_parameter(name_+".topic_image_raw", topic_image_raw_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "topic_image_raw: %s", topic_image_raw_.c_str());

  node_->declare_parameter(name_+".topic_image_info", rclcpp::ParameterValue("camera_info"));
  node_->get_parameter(name_+".topic_image_info", topic_image_info_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "topic_image_info: %s", topic_image_info_.c_str());

  node_->declare_parameter(name_+".detect_tag_frequency", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_+".detect_tag_frequency", detect_tag_frequency_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "detect_tag_frequency: %.1f", detect_tag_frequency_);

  tag_pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(name_+"/tag_pose", 2);
  
  result_image_pub_ = 
      node_->create_publisher<sensor_msgs::msg::Image>(name_+"/result_image", 1);

  auto loop_time = std::chrono::milliseconds(int(1000/detect_tag_frequency_));
  detect_tag_timer_ = node_->create_wall_timer(
    loop_time, std::bind(&AprilTagTracking::detectingLoop, this));
  
  stopDetection();
}

void AprilTagTracking::startDetection() {

  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    topic_image_raw_, rclcpp::QoS(1).best_effort(),
    std::bind(&AprilTagTracking::imageCallback, this, std::placeholders::_1));

  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    topic_image_info_, rclcpp::QoS(1).best_effort(),
    std::bind(&AprilTagTracking::cameraInfoCallback, this, std::placeholders::_1));

  detect_tag_timer_->reset();
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s initialized with best_effort QoS subscribers.", name_.c_str());

}

void AprilTagTracking::stopDetection() {

  image_sub_.reset();
  camera_info_sub_.reset();
  detect_tag_timer_->cancel();
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s is stopped.", name_.c_str());

}

AprilTagTracking::~AprilTagTracking() {

}

void AprilTagTracking::detectingLoop()
{

  if(msg_img_ == nullptr || msg_ci_ == nullptr){
    return;
  }

  try
  {
    cv_image_ = cv_bridge::toCvCopy(msg_img_, msg_img_->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(node_->get_logger().get_child(name_), "cv_bridge exception: %s", e.what());
    return;
  }
  geometry_msgs::msg::PoseStamped pose_out;
  tag_detector_->detectTags(cv_image_, msg_ci_, pose_out);

  //tag_detector_->drawDetections(cv_image_);
  //sensor_msgs::msg::Image::SharedPtr msg = cv_image_->toImageMsg();
  //result_image_pub_->publish(*msg);
  if(pose_out.header.frame_id!="")
    tag_pose_pub_->publish(pose_out);
}

void AprilTagTracking::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  msg_img_ = msg;
}

void AprilTagTracking::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (!img_info_get_) {
    msg_ci_ = msg;
    img_info_get_ = true;
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Received camera info");
  }
}

} // namespace dddmr_docking
