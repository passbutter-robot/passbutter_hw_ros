#include "../include/passbutter_driver/stepper_driver_node.hpp"

namespace passbutter_driver
{

StepperDriverNode::StepperDriverNode()
    : Node("stepper_driver")
{
    RCLCPP_INFO(this->get_logger(), "initialize node 'stepper_driver'");
}
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<passbutter_driver::StepperDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}