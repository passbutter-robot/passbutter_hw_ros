#include "../include/passbutter_driver/wheel_driver_node.hpp"
#include "ThunderBorg/include/ThunderBorg.hpp"

namespace passbutter_driver
{

WheelDriverNode::WheelDriverNode()
    : Node("wheel_driver")
{
    RCLCPP_INFO(this->get_logger(), "initialize node 'wheel_driver'");

    rcl_interfaces::msg::ParameterDescriptor bus_number_descriptor;
    bus_number_descriptor.name = "bus_number";
    bus_number_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    bus_number_descriptor.description = "the thunderborg i2c bus number";
    this->declare_parameter("bus_number", 0, bus_number_descriptor);

    this->declare_parameter("board_addresses");

    int busNumber = -1;
    this->get_parameter<int>("bus_number", busNumber);
    RCLCPP_INFO(this->get_logger(), "bus number=%d", busNumber);

    rclcpp::Parameter boardAdressesParam = this->get_parameter("board_addresses");
    std::vector<int64_t> boardAddresses = boardAdressesParam.as_integer_array();

    if (busNumber < 0 || boardAddresses.size() == 0)
    {
        busNumber = 1;
        RCLCPP_INFO(this->get_logger(), "try to autodetect board address on bus=%d", busNumber);
        auto wheelControl = new passbutter::MotorControl("wheel_control");
        auto addrs = wheelControl->detectBoards(busNumber);
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
            auto wheelControl = new passbutter::MotorControl("wheel_control");
            wheelControl->detectBoards(busNumber, boardAddresses[i], boardAddresses[i] + 1);
            wheelControl->initBus(boardAddresses[i]);
            this->wheelControls.push_back(wheelControl); 
        }
    }

    this->_speed = this->create_subscription<example_interfaces::msg::Float32>("wheel_speed", 10,
        std::bind(&WheelDriverNode::wheelCallback, this, std::placeholders::_1));
}

WheelDriverNode::~WheelDriverNode()
{
    for (int i = 0; i < this->wheelControls.size(); i++)
    {
        delete this->wheelControls[i];
    }
}

void WheelDriverNode::wheelCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received wheel callback: speed=%d", msg->data);

    for (int j = 0; j < this->wheelControls.size(); j++)
    {
        this->wheelControls[j]->setMotor1(msg->data);
        this->wheelControls[j]->setMotor2(msg->data);
    }
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<passbutter_driver::WheelDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}