#include "../include/passbutter_driver/single_stepper_driver_node.hpp"
#include "ThunderBorg/include/ThunderBorg.hpp"

namespace passbutter_driver
{

SingleStepperDriverNode::SingleStepperDriverNode()
    : Node("single_stepper_driver")
{
    RCLCPP_INFO(this->get_logger(), "initialize node 'single_stepper_driver'");

    this->_steps = this->create_subscription<example_interfaces::msg::Int32>("steps", 10,
            std::bind(&SingleStepperDriverNode::stepperCallback, this, std::placeholders::_1));

    this->stepperControl = new passbutter::StepperControl("stepper", 1, 0.3);
    std::vector<int> boardAddrs = this->stepperControl->detectBoards(1);
    if (boardAddrs.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "no boards detected..");
    }
    else
    {
        this->stepperControl->initBus(boardAddrs[0]);
    }
}

void SingleStepperDriverNode::stepperCallback(const example_interfaces::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received stepper callback: steps=%d", msg->data);

    bool backwards = msg->data < 0;

    for (int i = 0; i < std::abs(msg->data); i++)
    {
        this->stepperControl->move(backwards);
        usleep(500);
    }
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<passbutter_driver::SingleStepperDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}