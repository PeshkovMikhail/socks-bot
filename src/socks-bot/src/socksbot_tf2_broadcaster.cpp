#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class StaticFramePublisher : public rclcpp::Node
{
public:
  StaticFramePublisher()
  : Node("socks_bot_tf2_broadcaster")
  {
    
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_left_wheel_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_right_wheel_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&StaticFramePublisher::odom_callback, this, _1));

    left_angle_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "/left_wheel_angle", 10, std::bind(&StaticFramePublisher::left_wheel_callback, this, _1));
    right_angle_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "/right_wheel_angle", 10, std::bind(&StaticFramePublisher::right_wheel_callback, this, _1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry& msg) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    
    t.transform.translation.x = msg.pose.pose.position.x;
    t.transform.translation.y = msg.pose.pose.position.y;
    t.transform.translation.z = msg.pose.pose.position.z;
    t.transform.rotation = msg.pose.pose.orientation;

    tf_static_broadcaster_->sendTransform(t);
  }

  void left_wheel_callback(const std_msgs::msg::Float32& msg) {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "drivewhl_l_link";
    tf2::Quaternion q;
    q.setRPY(0, msg.data, 0);

    t.transform.translation.x = -0.065;
    t.transform.translation.y = 0.05;
    t.transform.translation.z = -0.03;

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_left_wheel_broadcaster_->sendTransform(t);
  }

  void right_wheel_callback(const std_msgs::msg::Float32& msg) {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "drivewhl_r_link";
    tf2::Quaternion q;
    q.setRPY(M_PI, msg.data, 0);

    t.transform.translation.x = -0.065;
    t.transform.translation.y = -0.05;
    t.transform.translation.z = -0.03;

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_right_wheel_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_left_wheel_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_right_wheel_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_angle_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_angle_subscription_;
};

int main(int argc, char * argv[])
{
  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>());
  rclcpp::shutdown();
  return 0;
}