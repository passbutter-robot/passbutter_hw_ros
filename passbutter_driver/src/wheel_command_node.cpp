#include "../include/passbutter_driver/wheel_command_node.hpp"

namespace passbutter_driver
{

WheelCommandNode::WheelCommandNode()
    : Node("wheel_command")
{
    RCLCPP_INFO(this->get_logger(), "initialize node 'wheel_command'");

    rcl_interfaces::msg::ParameterDescriptor target_speed_descriptor;
    target_speed_descriptor.name = "target_speed";
    target_speed_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    target_speed_descriptor.description = "the motor speed [0..1]";
    this->declare_parameter("target_speed", 0.0, target_speed_descriptor);

    rcl_interfaces::msg::ParameterDescriptor rate_seconds_descriptor;
    rate_seconds_descriptor.name = "rate_seconds";
    rate_seconds_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    rate_seconds_descriptor.description = "the amount of seconds after the next command is sent";
    this->declare_parameter("rate_seconds", 1, rate_seconds_descriptor);

    
    this->_target_speed = 0.0;
    this->get_parameter<double>("target_speed", this->_target_speed);

    int rateSeconds = 0;
    this->get_parameter<int>("rate_seconds", rateSeconds);

    
    this->_wheel_speed = this->create_publisher<example_interfaces::msg::Float32>("wheel_speed", this->_target_speed);
    this->_timer = this->create_wall_timer(std::chrono::seconds(rateSeconds),
            std::bind(&WheelCommandNode::timer_callback, this));
    this->_reverse = false;
}

void WheelCommandNode::timer_callback()
{
    auto msg = example_interfaces::msg::Float32();
    msg.data = (float)this->_target_speed;
    if (this->_reverse) msg.data *= -1;
    this->_reverse = !this->_reverse;

    RCLCPP_INFO(this->get_logger(), "send wheel speed=%d", msg.data);

    this->_wheel_speed->publish(msg);
}
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<passbutter_driver::WheelCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}