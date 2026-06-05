#include "apriltag_trt_tracking.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking {

AprilTagTrtTracking::AprilTagTrtTracking(rclcpp::Node* node, std::string name, bool record_tag) : node_(node), name_(name), 
img_info_get_(false){
  
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

  node_->declare_parameter(name_+".trt_model_path", rclcpp::ParameterValue(""));
  node_->get_parameter(name_+".trt_model_path", trt_model_path_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "trt_model_path: %s" , trt_model_path_.c_str());
  
  pub_annotated_img_ = node_->create_publisher<sensor_msgs::msg::Image>(name_+"/annotated_image", 1);

  if (std::filesystem::exists(trt_model_path_)) {
    is_trt_engine_exist_ = true;
  }
  
#ifdef TRT_ENABLED
  if(is_trt_engine_exist_){
    YoloV8Config config;
    config.segH = 200; //trained model size divide by 4
    config.segW = 200; //trained model size divide by 4
    yolov8_ = std::make_shared<YoloV8>("", trt_model_path_, config);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "%s load trt model: %s",name_.c_str(), trt_model_path_.c_str());
  }
  else{
    RCLCPP_ERROR(node_->get_logger().get_child(name_), "\033[1;31mtensorRT is specified but %s does not exist\033[0m", trt_model_path_.c_str());
  }
#endif

  auto loop_time = std::chrono::milliseconds(int(1000/detect_tag_frequency_));
  detect_tag_timer_ = node_->create_wall_timer(
    loop_time, std::bind(&AprilTagTrtTracking::detectingLoop, this));
  
  stopDetection();
}

void AprilTagTrtTracking::startDetection() {

  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    topic_image_raw_, rclcpp::QoS(1).best_effort(),
    std::bind(&AprilTagTrtTracking::imageCallback, this, std::placeholders::_1));

  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    topic_image_info_, rclcpp::QoS(1).best_effort(),
    std::bind(&AprilTagTrtTracking::cameraInfoCallback, this, std::placeholders::_1));

  detect_tag_timer_->reset();
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s initialized with best_effort QoS subscribers.", name_.c_str());

}

void AprilTagTrtTracking::stopDetection() {

  image_sub_.reset();
  camera_info_sub_.reset();
  detect_tag_timer_->cancel();
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s is stopped.", name_.c_str());

}

AprilTagTrtTracking::~AprilTagTrtTracking() {

}

void AprilTagTrtTracking::detectingLoop()
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

  // Run inference
  const auto objects = yolov8_->detectObjects(cv_image_->image);

  // Draw the bounding boxes on the image
  yolov8_->drawObjectLabels(cv_image_->image, objects);
  
  /*
  // Remove object from projected_image
  cv::Mat full_sized_mask = cv::Mat::zeros(inferenced_image.size(), CV_8UC1);
  for (const auto &object : objects) {

    //@ label=0 is people, remove it
    if(object.label==0){
      cv::Mat roi_mask = full_sized_mask(object.rect);
      object.boxMask.copyTo(roi_mask);
    }
    //cv::Size imageSize = object.boxMask.size();
    //RCLCPP_INFO(this->get_logger(), "object class: %d, confidence: %.2f", object.label, object.probability);
  }
  cv::Mat inverted_mask;
  cv::bitwise_not(full_sized_mask, inverted_mask);
  cv::bitwise_and(inferenced_image, inferenced_image, range_mat_removing_moving_object_, inverted_mask);
  */

  cv_bridge::CvImage img_annotated;
  img_annotated.image = cv_image_->image;
  //img_annotated.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  img_annotated.encoding = sensor_msgs::image_encodings::RGB8;
  sensor_msgs::msg::Image::SharedPtr ros2_annotated_img = img_annotated.toImageMsg();
  pub_annotated_img_->publish(*ros2_annotated_img);
}

void AprilTagTrtTracking::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  msg_img_ = msg;
}

void AprilTagTrtTracking::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (!img_info_get_) {
    msg_ci_ = msg;
    img_info_get_ = true;
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Received camera info");
  }
}

} // namespace dddmr_docking
