#include "../include/passbutter_driver/single_stepper_command_node.hpp"

namespace passbutter_driver
{

SingleStepperCommandNode::SingleStepperCommandNode()
    : Node("single_stepper_command")
{
    RCLCPP_INFO(this->get_logger(), "initialize node 'single_stepper_command'");

    rcl_interfaces::msg::ParameterDescriptor step_count_descriptor;
    step_count_descriptor.name = "step_count";
    step_count_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    step_count_descriptor.description = "the step count";
    this->declare_parameter("step_count", 0, step_count_descriptor);

    rcl_interfaces::msg::ParameterDescriptor rate_seconds_descriptor;
    rate_seconds_descriptor.name = "rate_seconds";
    rate_seconds_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    rate_seconds_descriptor.description = "the amount of seconds after the next command is sent";
    this->declare_parameter("rate_seconds", 1, rate_seconds_descriptor);

    
    this->_stepCount = 0;
    this->get_parameter<int>("step_count", this->_stepCount);

    int rateSeconds = 0;
    this->get_parameter<int>("rate_seconds", rateSeconds);

    
    this->_steps = this->create_publisher<example_interfaces::msg::Int32>("steps", this->_stepCount);
    this->_timer = this->create_wall_timer(std::chrono::seconds(rateSeconds),
            std::bind(&SingleStepperCommandNode::timer_callback, this));
    this->_reverse = false;
}

void SingleStepperCommandNode::timer_callback()
{
    auto msg = example_interfaces::msg::Int32();
    msg.data = this->_stepCount;
    if (this->_reverse) msg.data *= -1;
    this->_reverse = !this->_reverse;

    RCLCPP_INFO(this->get_logger(), "send steps=%d", msg.data);

    this->_steps->publish(msg);
}
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<passbutter_driver::SingleStepperCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}