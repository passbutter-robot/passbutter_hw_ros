#include "../include/passbutter_driver/single_stepper_driver_node.hpp"
#include "ThunderBorg/include/ThunderBorg.hpp"

namespace passbutter_driver
{

SingleStepperDriverNode::SingleStepperDriverNode()
    : Node("single_stepper_driver")
{
    RCLCPP_INFO(this->get_logger(), "initialize node 'single_stepper_driver'");

    rcl_interfaces::msg::ParameterDescriptor bus_number_descriptor;
    bus_number_descriptor.name = "bus_number";
    bus_number_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    bus_number_descriptor.description = "the thunderborg i2c bus number";
    this->declare_parameter("bus_number", 0, bus_number_descriptor);

    rcl_interfaces::msg::ParameterDescriptor board_address_descriptor;
    board_address_descriptor.name = "board_address";
    board_address_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    board_address_descriptor.description = "the thunderborg i2c board address";
    this->declare_parameter("board_address", 0, board_address_descriptor);

    int busNumber = -1;
    this->get_parameter<int>("bus_number", busNumber);

    int boardAddress = -1;
    this->get_parameter<int>("board_address", boardAddress);

    RCLCPP_INFO(this->get_logger(), "bus number=%d", busNumber);
    RCLCPP_INFO(this->get_logger(), "board address=%d", boardAddress);

    this->_steps = this->create_subscription<example_interfaces::msg::Int32>("steps", 10,
            std::bind(&SingleStepperDriverNode::stepperCallback, this, std::placeholders::_1));

    this->stepperControl = new passbutter::StepperControl("stepper", 0.3, 0.1);

    std::vector<int> boardAddrs;
    if (busNumber < 0 || boardAddress < 0)
    {
        busNumber = 1;
        RCLCPP_INFO(this->get_logger(), "try to autodetect board address on bus=%d", busNumber);
        boardAddrs = this->stepperControl->detectBoards(busNumber);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "try detect board address on bus=%d for address=%d", busNumber, boardAddress);
        boardAddrs = this->stepperControl->detectBoards(busNumber, boardAddress, boardAddress + 1);
    }

    if (boardAddrs.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "no boards detected..");
        boardAddress = boardAddrs[0];
        this->foundBoard = false;
    }
    else
    {
        this->stepperControl->initBus(boardAddrs[0]);
        this->foundBoard = true;

        this->stepperControl->initSteps();
    }
}

void SingleStepperDriverNode::stepperCallback(const example_interfaces::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received stepper callback: steps=%d", msg->data);

    bool backwards = msg->data < 0;

    for (int i = 0; i < std::abs(msg->data); i++)
    {
        this->stepperControl->move(backwards);
        rclcpp::sleep_for(std::chrono::milliseconds(this->stepDelayMS));
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