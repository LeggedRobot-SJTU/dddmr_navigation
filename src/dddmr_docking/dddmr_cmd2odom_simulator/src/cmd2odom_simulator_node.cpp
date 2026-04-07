#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>

class Cmd2OdomSimulatorNode : public rclcpp::Node
{
public:
  Cmd2OdomSimulatorNode() : Node("cmd2odom_simulator_node"),
    x_(0.0), y_(0.0), theta_(0.0), v_(0.0), omega_(0.0)
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Cmd2OdomSimulatorNode::cmdVelCallback, this, std::placeholders::_1));

    last_time_ = this->now();
    last_cmd_vel_time_ = this->now();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Update kinematics at 50Hz (20ms)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&Cmd2OdomSimulatorNode::updateKinematics, this));

    RCLCPP_INFO(this->get_logger(), "cmd2odom_simulator_node has been started.");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    v_ = msg->linear.x;
    omega_ = msg->angular.z;
    last_cmd_vel_time_ = this->now();
  }

  void updateKinematics()
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // Implement a simple timeout: stop robot if no cmd_vel received recently
    if ((current_time - last_cmd_vel_time_).seconds() > 0.5) {
      v_ = 0.0;
      omega_ = 0.0;
    }

    // 2D differential drive kinematics computations
    double delta_x = v_ * cos(theta_) * dt;
    double delta_y = v_ * sin(theta_) * dt;
    double delta_theta = omega_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    last_time_ = current_time;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v_;
    odom.twist.twist.angular.z = omega_;

    odom_pub_->publish(odom);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double x_, y_, theta_;
  double v_, omega_;
  rclcpp::Time last_time_;
  rclcpp::Time last_cmd_vel_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Cmd2OdomSimulatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
