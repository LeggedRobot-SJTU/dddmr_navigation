#include "apriltag_tracking.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking {

AprilTagTracking::AprilTagTracking(rclcpp::Node* node, std::string name) : node_(node), name_(name), 
img_info_get_(false),
td(apriltag_detector_create()) {
  
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

  // get method for estimating tag pose
  std::string pose_estimation_method;
  node_->declare_parameter(name_+".pose_estimation_method", rclcpp::ParameterValue("pnp"));
  node_->get_parameter(name_+".pose_estimation_method", pose_estimation_method);
  estimate_pose = pose_estimation_methods.at(pose_estimation_method);

  // detector parameters in "detector" namespace
  node_->declare_parameter(name_+".detector.threads", rclcpp::ParameterValue(td->nthreads));
  node_->get_parameter(name_+".detector.threads", td->nthreads);

  node_->declare_parameter(name_+".detector.decimate", rclcpp::ParameterValue(td->quad_decimate));
  node_->get_parameter(name_+".detector.decimate", td->quad_decimate);

  node_->declare_parameter(name_+".detector.blur", rclcpp::ParameterValue(td->quad_sigma));
  node_->get_parameter(name_+".detector.blur", td->quad_sigma);

  node_->declare_parameter(name_+".detector.refine", rclcpp::ParameterValue(td->refine_edges));
  node_->get_parameter(name_+".detector.refine", td->refine_edges);

  node_->declare_parameter(name_+".detector.sharpening", rclcpp::ParameterValue(td->decode_sharpening));
  node_->get_parameter(name_+".detector.sharpening", td->decode_sharpening);

  node_->declare_parameter(name_+".detector.debug", rclcpp::ParameterValue(td->debug));
  node_->get_parameter(name_+".detector.debug", td->debug);

  int max_hamming_val;
  node_->declare_parameter(name_+".max_hamming", rclcpp::ParameterValue(0));
  node_->get_parameter(name_+".max_hamming", max_hamming_val);
  max_hamming = max_hamming_val;

  bool profile_val;
  node_->declare_parameter(name_+".profile", rclcpp::ParameterValue(false));
  node_->get_parameter(name_+".profile", profile_val);
  profile = profile_val;

  if(!sizes.empty()) {
      // use tag specific size
      if(ids.size() != sizes.size()) {
          throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
      }
      for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
  }

  if(tag_fun.count(tag_family_)) {
      tf = tag_fun.at(tag_family_).first();
      tf_destructor = tag_fun.at(tag_family_).second;
      apriltag_detector_add_family(td, tf);
  }
  else {
      throw std::runtime_error("Unsupported tag family: " + tag_family_);
  }
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

  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    topic_image_raw_, rclcpp::QoS(1).best_effort(),
    std::bind(&AprilTagTracking::imageCallback, this, std::placeholders::_1));

  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    topic_image_info_, rclcpp::QoS(1).best_effort(),
    std::bind(&AprilTagTracking::cameraInfoCallback, this, std::placeholders::_1));
  
  auto loop_time = std::chrono::milliseconds(int(1000/detect_tag_frequency_));
  detect_tag_timer_ = node_->create_wall_timer(
    loop_time, std::bind(&AprilTagTracking::detectingLoop, this));

  RCLCPP_INFO(node_->get_logger().get_child(name_), "AprilTagTracking initialized with best_effort QoS subscribers.");
}

AprilTagTracking::~AprilTagTracking() {
  apriltag_detector_destroy(td);
  tf_destructor(tf);
}

void AprilTagTracking::detectingLoop()
{
  if(msg_img_ != nullptr && msg_ci_ != nullptr){

  const std::array<double, 4> intrinsics = {msg_ci_->p.data()[0], msg_ci_->p.data()[5], msg_ci_->p.data()[2], msg_ci_->p.data()[6]};

  // convert to 8bit monochrome image
  const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img_, "mono8")->image;

  image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

  #ifdef HAVE_SYS_TIME_H
  struct timeval start, end;
  double start_t, end_t, t_diff;
  gettimeofday(&start, NULL);
  #endif

  zarray_t* detections = apriltag_detector_detect(td, &im);

  #ifdef HAVE_SYS_TIME_H
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  RCLCPP_WARN(node_->get_logger().get_child(name_), "Map update time: %.9f", t_diff);
  #endif


  if(profile)
      timeprofile_display(td->tp);

  for(int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);

      RCLCPP_DEBUG(node_->get_logger().get_child(name_),
                  "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                  i, det->family->nbits, det->family->h, det->id,
                  det->hamming, det->decision_margin);

      // ignore untracked tags
      if(!tag_frames.empty() && !tag_frames.count(det->id)) { 
        continue; }

      // reject detections with more corrected bits than allowed
      if(det->hamming > max_hamming) { 
        continue; }

      // detection

      // 3D orientation and position
      geometry_msgs::msg::TransformStamped tf;
      tf.header = msg_img_->header;
      // set child frame name by generic tag name or configured tag name
      tf.child_frame_id = name_;
      const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
      if(estimate_pose != nullptr) {
          tf.transform = estimate_pose(det, intrinsics, size);
      }


      geometry_msgs::msg::PoseStamped before_pose;
      before_pose.header = tf.header;
      before_pose.pose.position.x = tf.transform.translation.x;
      before_pose.pose.position.y = tf.transform.translation.y;
      before_pose.pose.position.z = tf.transform.translation.z;
      before_pose.pose.orientation.x = tf.transform.rotation.x;
      before_pose.pose.orientation.y = tf.transform.rotation.y;
      before_pose.pose.orientation.z = tf.transform.rotation.z;
      before_pose.pose.orientation.w = tf.transform.rotation.w;
      pose_pub_->publish(before_pose);

      // origin tf
      tf2::Transform tf2_origin;
      tf2_origin.setOrigin(tf2::Vector3(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
      tf2_origin.setRotation(tf2::Quaternion(
      tf.transform.rotation.x, tf.transform.rotation.y, 
      tf.transform.rotation.z, tf.transform.rotation.w));

      // use the tf2_static which rotate to our desire direction

      // after tf
      tf2::Transform tf2_after;
      tf2_after.mult(tf2_origin,tf2_static_rotate_);

      //filter stuff (stand at camera frame ,filter pitch)
      tf2::Quaternion tmp_quaternion(tf2_after.getRotation().x(), tf2_after.getRotation().y(), tf2_after.getRotation().z(), tf2_after.getRotation().w());
      tf2::Matrix3x3 tmp_matrix_trans(tmp_quaternion);
      tmp_matrix_trans.getRPY(current_roll_,  current_pitch_, current_yaw_);

      double diff_yaw = angles::shortest_angular_distance(current_pitch_, last_pitch_);

      //RCLCPP_WARN(this->get_logger(), " stand at camera optical frame -> yaw diff: %.3f, current yaw: %.3f , last yaw: %.3f", diff_yaw , current_pitch_, last_pitch_);
      
      if(!is_initial_){
        last_roll_ = current_roll_;
        last_pitch_ = current_pitch_;
        last_yaw_ = current_yaw_;
        is_initial_ = true;
      }
      if(fabs(diff_yaw)> 0.2){
        RCLCPP_WARN(this->get_logger(), "Before current diff: %.10f", current_pitch_ );
        current_pitch_ = (last_pitch_*0.97) + (current_pitch_*0.03);
        RCLCPP_WARN(this->get_logger(), "After current diff: %.10f", current_pitch_ );
        RCLCPP_WARN(this->get_logger(), "----------------------");
      }
      last_roll_ = current_roll_;
      last_pitch_ = current_pitch_;
      last_yaw_ = current_yaw_;

      tf2::Quaternion rorateQuaternion_do_filter;
      rorateQuaternion_do_filter.setRPY(current_roll_, current_pitch_, current_yaw_);

      geometry_msgs::msg::PoseStamped after_pose;
      after_pose.header = tf.header;
      after_pose.pose.position.x = tf2_after.getOrigin().x();
      after_pose.pose.position.y = tf2_after.getOrigin().y();
      after_pose.pose.position.z = tf2_after.getOrigin().z();
      //after_pose.pose.orientation.x =  tf2_after.getRotation().x();
      //after_pose.pose.orientation.y =  tf2_after.getRotation().y();
      //after_pose.pose.orientation.z =  tf2_after.getRotation().z();
      //after_pose.pose.orientation.w =  tf2_after.getRotation().w();
      after_pose.pose.orientation.x = rorateQuaternion_do_filter.x();
      after_pose.pose.orientation.y = rorateQuaternion_do_filter.y();
      after_pose.pose.orientation.z = rorateQuaternion_do_filter.z();
      after_pose.pose.orientation.w = rorateQuaternion_do_filter.w();
      pose_pub_after_->publish(after_pose);

    }

    apriltag_detections_destroy(detections);
    //RCLCPP_ERROR(this->get_logger(), "L end");
  }
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
