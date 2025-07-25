#ifndef SOCKS_BOT__SOCKS_BOT_HARDWARE_HPP_
#define SOCKS_BOT__SOCKS_BOT_HARDWARE_HPP_

// #include "socks_bot/visibility_control.h"

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace socks_bot
{
class SocksBotHardware  : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SocksBotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;

  // Joint data
  double left_pos_ = 0.0, right_pos_ = 0.0;
  double left_vel_ = 0.0, right_vel_ = 0.0;
  double left_cmd_ = 0.0, right_cmd_ = 0.0;
  float wheel_radius_ = 0.0, wheel_separation_ = 0.0;

  void wheel_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
};

}  // namespace socks_bot

#endif  // SOCKS_BOT__SOCKS_BOT_HARDWARE_HPP_
