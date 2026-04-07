#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TagPosePublisher : public rclcpp::Node
{
public:
  TagPosePublisher() : Node("tag_pose_publisher")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tag_pose", 10);

    timer_ = this->create_wall_timer(
      50ms, std::bind(&TagPosePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_->lookupTransform("camera", "tag", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = transformStamped.header;
    pose_msg.pose.position.x = transformStamped.transform.translation.x;
    pose_msg.pose.position.y = transformStamped.transform.translation.y;
    pose_msg.pose.position.z = transformStamped.transform.translation.z;
    pose_msg.pose.orientation = transformStamped.transform.rotation;

    publisher_->publish(pose_msg);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
