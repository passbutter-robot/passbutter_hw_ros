#include "../include/passbutter_driver/single_stepper_command_node.hpp"

namespace passbutter_driver
{

SingleStepperCommandNode::SingleStepperCommandNode()
    : Node("single_stepper_command")
{
    RCLCPP_INFO(this->get_logger(), "initialize node 'single_stepper_command'");

    this->_steps = this->create_publisher<example_interfaces::msg::Int32>("steps", 10);
    this->_timer = this->create_wall_timer(std::chrono::seconds(5),
            std::bind(&SingleStepperCommandNode::timer_callback, this));
    this->_last_step_count = -50;
}

void SingleStepperCommandNode::timer_callback()
{
    auto msg = example_interfaces::msg::Int32();
    msg.data = this->_last_step_count;
    if (msg.data > 0) msg.data *= -1;
    this->_last_step_count = msg.data;

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