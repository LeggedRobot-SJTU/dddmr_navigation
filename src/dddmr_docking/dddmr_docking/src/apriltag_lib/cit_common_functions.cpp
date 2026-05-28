/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_lib/cit_common_functions.h"
#include "image_geometry/pinhole_camera_model.h"

#include "common/homography.h"
#include "tagStandard52h13.h"
#include "tagStandard41h12.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCustom48h12.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
//@ for mkdir
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace apriltag_ros
{

inline std::string currentDateTime() {
  std::time_t t = std::time(nullptr);
  std::tm *now = std::localtime(&t);

  char buffer[128];
  strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", now);
  return buffer;
}

TagDetector::TagDetector(std::string family, int thread, double decimate, 
                        double blur, bool refine_edges, double decode_sharpening, bool debug, int max_hamming_distance, std::map<int, double> id_size_map) :
    family_(family),
    threads_(thread),
    decimate_(decimate),
    blur_(blur),
    refine_edges_(refine_edges),
    decode_sharpening_(decode_sharpening),
    debug_(debug),
    max_hamming_distance_(max_hamming_distance),
    id_size_map_(id_size_map)
{

  mapping_dir_string_ = std::string("/tmp/") + currentDateTime();
  std::filesystem::create_directory(mapping_dir_string_);
  // static rotate tf
  tf2::Quaternion rorateQuaternion;
  rorateQuaternion.setRPY(-1.5707963, 1.5707963, 0.0);
  tf2_static_rotate_.setRotation(rorateQuaternion);
  tf2_static_rotate_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

  remove_duplicates_ = true;
  
  // Define the tag family whose tags should be searched for in the camera
  // images
  if (family_ == "tagStandard52h13")
  {
    tf_ = tagStandard52h13_create();
  }
  else if (family_ == "tagStandard41h12")
  {
    tf_ = tagStandard41h12_create();
  }
  else if (family_ == "tag36h11")
  {
    tf_ = tag36h11_create();
  }
  else if (family_ == "tag25h9")
  {
    tf_ = tag25h9_create();
  }
  else if (family_ == "tag16h5")
  {
    tf_ = tag16h5_create();
  }
  else if (family_ == "tagCustom48h12")
  {
    tf_ = tagCustom48h12_create();
  }
  else if (family_ == "tagCircle21h7")
  {
    tf_ = tagCircle21h7_create();
  }
  else if (family_ == "tagCircle49h12")
  {
    tf_ = tagCircle49h12_create();
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("cit_common_functions"), "Invalid tag family specified! Aborting");
    exit(1);
  }

  // Create the AprilTag 2 detector
  td_ = apriltag_detector_create();
  apriltag_detector_add_family_bits(td_, tf_, max_hamming_distance_);
  td_->quad_decimate = (float)decimate_;
  td_->quad_sigma = (float)blur_;
  td_->nthreads = threads_;
  td_->debug = debug_;
  td_->refine_edges = refine_edges_;
  td_->decode_sharpening = (float)decode_sharpening_;
  detections_ = NULL;
}

// destructor
TagDetector::~TagDetector() {
  // free memory associated with tag detector
  apriltag_detector_destroy(td_);

  // Free memory associated with the array of tag detections
  apriltag_detections_destroy(detections_);

  // free memory associated with tag family
  if (family_ == "tagStandard52h13")
  {
    tagStandard52h13_destroy(tf_);
  }
  else if (family_ == "tagStandard41h12")
  {
    tagStandard41h12_destroy(tf_);
  }
  else if (family_ == "tag36h11")
  {
    tag36h11_destroy(tf_);
  }
  else if (family_ == "tag25h9")
  {
    tag25h9_destroy(tf_);
  }
  else if (family_ == "tag16h5")
  {
    tag16h5_destroy(tf_);
  }
  else if (family_ == "tagCustom48h12")
  {
    tagCustom48h12_destroy(tf_);
  }
  else if (family_ == "tagCircle21h7")
  {
    tagCircle21h7_destroy(tf_);
  }
  else if (family_ == "tagCircle49h12")
  {
    tagCircle49h12_destroy(tf_);
  }
}

void TagDetector::detectTags (
    const cv_bridge::CvImagePtr& image,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info,
    geometry_msgs::msg::PoseStamped& pose_out) {
  // Convert image to AprilTag code's format
  cv::Mat gray_image;
  if (image->image.channels() == 1)
  {
    gray_image = image->image;
  }
  else
  {
    cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);
  }
  image_u8_t apriltag_image = { .width = gray_image.cols,
                                  .height = gray_image.rows,
                                  .stride = gray_image.cols,
                                  .buf = gray_image.data
  };

  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);

  // Get camera intrinsic properties for rectified image.
  double fx = camera_model.fx(); // focal length in camera x-direction [px]
  double fy = camera_model.fy(); // focal length in camera y-direction [px]
  double cx = camera_model.cx(); // optical center x-coordinate [px]
  double cy = camera_model.cy(); // optical center y-coordinate [px]

  // Run AprilTag 2 algorithm on the image
  if (detections_)
  {
    apriltag_detections_destroy(detections_);
    detections_ = NULL;
  }
  detections_ = apriltag_detector_detect(td_, &apriltag_image);

  // If remove_dulpicates_ is set to true, then duplicate tags are not allowed.
  // Thus any duplicate tag IDs visible in the scene must include at least 1
  // erroneous detection. Remove any tags with duplicate IDs to ensure removal
  // of these erroneous detections
  if (remove_duplicates_)
  {
    removeDuplicates();
  }

  for (int i=0; i < zarray_size(detections_); i++)
  {
    // Get the i-th detected tag
    apriltag_detection_t *detection;
    zarray_get(detections_, i, &detection);

    // Bootstrap this for loop to find this tag's description amongst
    // the tag bundles. If found, add its points to the bundle's set of
    // object-image corresponding points (tag corners) for cv::solvePnP.
    // Don't yet run cv::solvePnP on the bundles, though, since we're still in
    // the process of collecting all the object-image corresponding points
    int tagID = detection->id;
    bool is_part_of_bundle = false;
    //RCLCPP_WARN(rclcpp::get_logger("cit_common_functions"), "ID: %d", tagID);
    //=================================================================
    // The remainder of this for loop is concerned with standalone tag
    // poses!
    double tag_size = 0;
    if(id_size_map_.count(detection->id))
      tag_size = id_size_map_.at(detection->id);
    else
      continue;

    // Get estimated tag pose in the camera frame.
    //
    // Note on frames:
    // The raw AprilTag 2 uses the following frames:
    //   - camera frame: looking from behind the camera (like a
    //     photographer), x is right, y is up and z is towards you
    //     (i.e. the back of camera)
    //   - tag frame: looking straight at the tag (oriented correctly),
    //     x is right, y is down and z is away from you (into the tag).
    // But we want:
    //   - camera frame: looking from behind the camera (like a
    //     photographer), x is right, y is down and z is straight
    //     ahead
    //   - tag frame: looking straight at the tag (oriented correctly),
    //     x is right, y is up and z is towards you (out of the tag).
    // Using these frames together with cv::solvePnP directly avoids
    // AprilTag 2's frames altogether.
    // TODO solvePnP[Ransac] better?
    std::vector<cv::Point3d > standaloneTagObjectPoints;
    std::vector<cv::Point2d > standaloneTagImagePoints;
    addObjectPoints(tag_size/2, cv::Matx44d::eye(), standaloneTagObjectPoints);
    addImagePoints(detection, standaloneTagImagePoints);
    Eigen::Matrix4d transform = getRelativeTransform(standaloneTagObjectPoints,
                                                     standaloneTagImagePoints,
                                                     fx, fy, cx, cy);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion(rot);

    makeTagPose(transform, rot_quaternion, image->header, pose_out);

  }

}

int TagDetector::idComparison (const void* first, const void* second)
{
  int id1 = ((apriltag_detection_t*) first)->id;
  int id2 = ((apriltag_detection_t*) second)->id;
  return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
}

void TagDetector::removeDuplicates ()
{
  zarray_sort(detections_, &idComparison);
  int count = 0;
  bool duplicate_detected = false;
  while (true)
  {
    if (count > zarray_size(detections_)-1)
    {
      // The entire detection set was parsed
      return;
    }
    apriltag_detection_t *detection;
    zarray_get(detections_, count, &detection);
    int id_current = detection->id;
    // Default id_next value of -1 ensures that if the last detection
    // is a duplicated tag ID, it will get removed
    int id_next = -1;
    if (count < zarray_size(detections_)-1)
    {
      zarray_get(detections_, count+1, &detection);
      id_next = detection->id;
    }
    if (id_current == id_next || (id_current != id_next && duplicate_detected))
    {
      duplicate_detected = true;
      // Remove the current tag detection from detections array
      int shuffle = 0;
      zarray_remove_index(detections_, count, shuffle);
      if (id_current != id_next)
      {
        RCLCPP_WARN(rclcpp::get_logger("cit_common_functions"), "Pruning tag ID %d because it appears more than once in the image.", id_current);
        duplicate_detected = false; // Reset
      }
      continue;
    }
    else
    {
      count++;
    }
  }
}

void TagDetector::addObjectPoints (
    double s, cv::Matx44d T_oi, std::vector<cv::Point3d >& objectPoints) const
{
  // Add to object point vector the tag corner coordinates in the bundle frame
  // Going counterclockwise starting from the bottom left corner
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s,-s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s,-s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s, s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s, s, 0, 1));
}

void TagDetector::addImagePoints (
    apriltag_detection_t *detection,
    std::vector<cv::Point2d >& imagePoints) const
{
  // Add to image point vector the tag corners in the image frame
  // Going counterclockwise starting from the bottom left corner
  double tag_x[4] = {-1,1,1,-1};
  double tag_y[4] = {1,1,-1,-1}; // Negated because AprilTag tag local
                                 // frame has y-axis pointing DOWN
                                 // while we use the tag local frame
                                 // with y-axis pointing UP
  for (int i=0; i<4; i++)
  {
    // Homography projection taking tag local frame coordinates to image pixels
    double im_x, im_y;
    homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
    imagePoints.push_back(cv::Point2d(im_x, im_y));
  }
}

Eigen::Matrix4d TagDetector::getRelativeTransform(
    std::vector<cv::Point3d > objectPoints,
    std::vector<cv::Point2d > imagePoints,
    double fx, double fy, double cx, double cy) const
{
  // perform Perspective-n-Point camera pose estimation using the
  // above 3D-2D point correspondences
  cv::Mat rvec, tvec;
  cv::Matx33d cameraMatrix(fx,  0, cx,
                           0,  fy, cy,
                           0,   0,  1);
  cv::Vec4f distCoeffs(0,0,0,0); // distortion coefficients
  // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
  // need to first check WHAT is a bottleneck in this code, and only
  // do this if PnP solution is the bottleneck.
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d wRo;
  wRo << R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2);

  Eigen::Matrix4d T; // homogeneous transformation matrix
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) <<
      tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0,0,0,1;
  return T;
}

void TagDetector::makeTagPose(
    const Eigen::Matrix4d& transform,
    const Eigen::Quaternion<double> rot_quaternion,
    const std_msgs::msg::Header& header,
    geometry_msgs::msg::PoseStamped& pose_out)
{
  tf2::Transform original_tf2;
  original_tf2.setRotation(tf2::Quaternion(rot_quaternion.x(), rot_quaternion.y(), rot_quaternion.z(), rot_quaternion.w()));
  original_tf2.setOrigin(tf2::Vector3(transform(0, 3), transform(1, 3), transform(2, 3)));
  
  tf2::Transform rotated_tf2;
  rotated_tf2.mult(original_tf2, tf2_static_rotate_);

  pose_out.header = header;
  //===== Position and orientation
  pose_out.pose.position.x    = rotated_tf2.getOrigin().x();
  pose_out.pose.position.y    = rotated_tf2.getOrigin().y();
  pose_out.pose.position.z    = rotated_tf2.getOrigin().z();
  pose_out.pose.orientation.x = rotated_tf2.getRotation().x();
  pose_out.pose.orientation.y = rotated_tf2.getRotation().y();
  pose_out.pose.orientation.z = rotated_tf2.getRotation().z();
  pose_out.pose.orientation.w = rotated_tf2.getRotation().w();
  //RCLCPP_WARN(rclcpp::get_logger("cit_common_functions"), "%.2f, %.2f, %.2f", pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);

}

void TagDetector::drawDetections (cv_bridge::CvImagePtr image, bool save_drawing)
{

  cv_bridge::CvImagePtr deep_copied_ptr(new cv_bridge::CvImage());

  deep_copied_ptr->header = image->header;
  deep_copied_ptr->encoding = image->encoding;
  deep_copied_ptr->image = image->image.clone(); // This creates the deep copy of the cv::Mat

  //@ generate labelled data stream
  std::string ids_yolo_format;
  for (int i = 0; i < zarray_size(detections_); i++)
  {
    apriltag_detection_t *det;
    zarray_get(detections_, i, &det);

    // Check if this ID is present in config/tags.yaml
    // Check if is part of a tag bundle
    int tagID = det->id;
    bool is_part_of_bundle = false;
    for (unsigned int j=0; j<tag_bundle_descriptions_.size(); j++)
    {
      TagBundleDescription bundle = tag_bundle_descriptions_[j];
      if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end())
      {
        is_part_of_bundle = true;
        break;
      }
    }

    // Draw tag outline with edge colors green, blue, blue, red
    // (going counter-clockwise, starting from lower-left corner in
    // tag coords). cv::Scalar(Blue, Green, Red) format for the edge
    // colors!
    line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
         cv::Point((int)det->p[1][0], (int)det->p[1][1]),
         cv::Scalar(0, 0xff, 0)); // green
    line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
         cv::Point((int)det->p[3][0], (int)det->p[3][1]),
         cv::Scalar(0, 0, 0xff)); // red
    line(image->image, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
         cv::Point((int)det->p[2][0], (int)det->p[2][1]),
         cv::Scalar(0xff, 0, 0)); // blue
    line(image->image, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
         cv::Point((int)det->p[3][0], (int)det->p[3][1]),
         cv::Scalar(0xff, 0, 0)); // blue

    // Print tag ID in the middle of the tag
    std::stringstream ss;
    ss << det->id;
    cv::String text = ss.str();
    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 0.5;
    int baseline;
    cv::Size textsize = cv::getTextSize(text, fontface,
                                        fontscale, 2, &baseline);
    cv::putText(image->image, text,
                cv::Point((int)(det->c[0]-textsize.width/2),
                          (int)(det->c[1]+textsize.height/2)),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
    

    /*
    Oriented Bounding Box (OBB) format
    class_index x1 y1 x2 y2 x3 y3 x4 y4
    The four corner points must be listed in clockwise order:
    (Top-Left -> Top-Right -> Bottom-Right -> Bottom-Left) relative to the object's orientation
    */
    double x1 = (double)det->p[3][0]/(double)image->image.cols;
    double y1 = (double)det->p[3][1]/(double)image->image.rows;

    double x2 = (double)det->p[2][0]/(double)image->image.cols;
    double y2 = (double)det->p[2][1]/(double)image->image.rows;

    double x3 = (double)det->p[1][0]/(double)image->image.cols;
    double y3 = (double)det->p[1][1]/(double)image->image.rows;

    double x4 = (double)det->p[0][0]/(double)image->image.cols;
    double y4 = (double)det->p[0][1]/(double)image->image.rows;

    ids_yolo_format = std::to_string(tagID) + " " + \
                        std::to_string(x1) + " " + std::to_string(y1) + " " +
                        std::to_string(x2) + " " + std::to_string(y2) + " " +
                        std::to_string(x3) + " " + std::to_string(y3) + " " +
                        std::to_string(x4) + " " + std::to_string(y4) + "\n";
    
  }

  //@ write image and labelled stream (yolov11)
  if(zarray_size(detections_)>0){
    std::string timestamp;
    std::stringstream ss;
    ss << image->header.stamp.sec << "_" << std::setw(9) << std::setfill('0') << image->header.stamp.nanosec;
    timestamp = ss.str();
    //to make the image name include family and tag_id
    std::string id_all = "";
    for (int i = 0; i < zarray_size(detections_); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections_, i, &det);
      int tagID = det->id;
      id_all = id_all + "_" + std::to_string(tagID);
    }
    std::string spec = family_ + id_all;
    std::string file_name = mapping_dir_string_ + "/" + timestamp + "_" + spec;
    
    cv::cvtColor(deep_copied_ptr->image, deep_copied_ptr->image, CV_BGR2RGB);
    cv::imwrite(file_name + ".png", deep_copied_ptr->image);
    
    RCLCPP_WARN(rclcpp::get_logger("cit_common_functions"), "Yolo label: %s", ids_yolo_format.c_str());

    std::ofstream outFile(file_name + ".txt");
    // 2. Check if the file opened successfully
    if (outFile.is_open()) {
        outFile << ids_yolo_format;
        outFile.close();
    } else {
        std::cerr << "Unable to open file!" << std::endl;
    }

  }
}


} // namespace apriltag_ros
