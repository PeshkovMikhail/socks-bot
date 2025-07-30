#include "socks_bot/socks_bot_hardware.hpp"

namespace socks_bot
{
hardware_interface::CallbackReturn SocksBotHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;

    wheel_separation_ = std::stof(info.hardware_parameters.at("wheel_separation"));
    wheel_radius_ = std::stof(info.hardware_parameters.at("wheel_radius"));


    node_ = rclcpp::Node::make_shared("socks_bot_diff_drive");
    command_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_rpm", 10);
    wheel_state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>("wheel_state",
        10, std::bind(&socks_bot::SocksBotHardware::wheel_state_callback, this, std::placeholders::_1));
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SocksBotHardware::export_state_interfaces()
{
    return {
        { "wheel_left_joint", hardware_interface::HW_IF_POSITION, &left_pos_ },
        { "wheel_left_joint", hardware_interface::HW_IF_VELOCITY, &left_vel_ },
        { "wheel_right_joint", hardware_interface::HW_IF_POSITION, &right_pos_ },
        { "wheel_right_joint", hardware_interface::HW_IF_VELOCITY, &right_vel_ }
    };
}

std::vector<hardware_interface::CommandInterface> SocksBotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("wheel_left_joint", hardware_interface::HW_IF_VELOCITY, &left_cmd_);
  command_interfaces.emplace_back("wheel_right_joint", hardware_interface::HW_IF_VELOCITY, &right_cmd_);
  return command_interfaces;
}


hardware_interface::CallbackReturn SocksBotHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(node_->get_logger(), "Activating hardware");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SocksBotHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(node_->get_logger(), "Deactivating hardware");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SocksBotHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    rclcpp::spin_some(node_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SocksBotHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    std_msgs::msg::Float32MultiArray cmd;
    cmd.data = { left_cmd_, right_cmd_ };
    command_pub_->publish(cmd);
    return hardware_interface::return_type::OK;
}

void SocksBotHardware::wheel_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    float left_wheel_rpm = msg->data[0];
    float left_wheel_angle = msg->data[1];
    float right_wheel_rpm = msg->data[2];
    float right_wheel_angle = msg->data[3];

    left_pos_ = left_wheel_angle;
    left_vel_ = left_wheel_rpm;
    right_pos_ = right_wheel_angle;
    right_vel_ = right_wheel_rpm;
}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  socks_bot::SocksBotHardware, hardware_interface::SystemInterface)
