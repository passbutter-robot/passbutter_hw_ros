#include "../include/passbutter_driver/stepper_driver_node.hpp"
#include "ThunderBorg/include/ThunderBorg.hpp"

namespace passbutter_driver
{

StepperDriverNode::StepperDriverNode()
    : Node("stepper_driver")
{
    RCLCPP_INFO(this->get_logger(), "initialize node 'stepper_driver'");

    rcl_interfaces::msg::ParameterDescriptor bus_number_descriptor;
    bus_number_descriptor.name = "bus_number";
    bus_number_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    bus_number_descriptor.description = "the thunderborg i2c bus number";
    this->declare_parameter("bus_number", 0, bus_number_descriptor);

    rcl_interfaces::msg::ParameterDescriptor step_delay_ms_descriptor;
    step_delay_ms_descriptor.name = "step_delay_ms";
    step_delay_ms_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    step_delay_ms_descriptor.description = "the delay between two steps in milliseconds";
    this->declare_parameter("step_delay_ms", 5, step_delay_ms_descriptor);

    rcl_interfaces::msg::ParameterDescriptor max_power_descriptor;
    max_power_descriptor.name = "max_power";
    max_power_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    max_power_descriptor.description = "the maximum power between 0..1";
    this->declare_parameter("max_power", 0.0, max_power_descriptor);

    rcl_interfaces::msg::ParameterDescriptor holding_power_descriptor;
    holding_power_descriptor.name = "holding_power";
    holding_power_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    holding_power_descriptor.description = "the holding power between 0..1";
    this->declare_parameter("holding_power", 0.0, holding_power_descriptor);

    this->declare_parameter("board_addresses");
    this->set_parameters({rclcpp::Parameter("board_addresses", std::vector<uint8_t>({0}))});

    int busNumber = -1;
    this->get_parameter<int>("bus_number", busNumber);
    RCLCPP_INFO(this->get_logger(), "bus number=%d", busNumber);

    this->get_parameter<int>("step_delay_ms", this->_stepDelayMS);
    RCLCPP_INFO(this->get_logger(), "step delay ms=%d", this->_stepDelayMS);

    double maxPower = 0;
    this->get_parameter<double>("max_power", maxPower);
    RCLCPP_INFO(this->get_logger(), "max power=%f", maxPower);

    double holdingPower = -1;
    this->get_parameter<double>("holding_power", holdingPower);
    RCLCPP_INFO(this->get_logger(), "holding power=%f", holdingPower);

    std::vector<uint8_t> boardAddresses;
    this->get_parameter("board_addresses", boardAddresses);
    
    if (busNumber < 0 || boardAddresses.size() == 0)
    {
        busNumber = 1;
        RCLCPP_INFO(this->get_logger(), "try to autodetect board address on bus=%d", busNumber);
        auto stepperControl = new passbutter::StepperControl("stepper", maxPower, holdingPower);
        auto addrs = stepperControl->detectBoards(busNumber);
        if (addrs.size() > 0)
        {
            std::copy(addrs.begin(), addrs.end(), back_inserter(boardAddresses));
        }
    }

    if (boardAddresses.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "no board addresses given or detected..");
    }
    else
    {
        for (int i = 0; i < boardAddresses.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "board address %d=%d", i, boardAddresses[i]);
            auto stepperControl = new passbutter::StepperControl("stepper", maxPower, holdingPower);
            stepperControl->detectBoards(busNumber, boardAddresses[i], boardAddresses[i] + 1);
            stepperControl->initBus(boardAddresses[i]);
            stepperControl->initSteps();
            // TODO: power off stepper
            this->stepperControls.push_back(stepperControl);
        }
    }

    this->_steps = this->create_subscription<example_interfaces::msg::Int32>("steps", 10,
        std::bind(&StepperDriverNode::stepperCallback, this, std::placeholders::_1));
}

void StepperDriverNode::stepperCallback(const example_interfaces::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received stepper callback: steps=%d", msg->data);

    bool backwards = msg->data < 0;

    for (int i = 0; i < this->stepperControls.size(); i++)
    {
        for (int j = 0; j < std::abs(msg->data); j++)
        {
            this->stepperControls[i]->move(backwards);
            rclcpp::sleep_for(std::chrono::milliseconds(this->_stepDelayMS));
        }
    }
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