#include "../include/passbutter_driver/stepper_hw_driver.hpp"
#include "ThunderBorg/include/ThunderBorg.hpp"
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <exception>

namespace passbutter_driver
{

StepperDriver::StepperDriver()
    : hardware_interface::BaseInterface<hardware_interface::SystemInterface>()
{
    this->stepperControl = new passbutter::StepperControl("stepper", 1, 0.3);
}

StepperDriver::~StepperDriver()
{
    delete this->stepperControl;
}

hardware_interface::return_type StepperDriver::configure(const hardware_interface::HardwareInfo & info)
{
    if (configure_default(info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }

    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    std::vector<int> boardAddrs = stepperControl->detectBoards(1);
    if (boardAddrs.size() == 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("StepperDriver"), "unable to detect any thunderborg controller..");
        return hardware_interface::return_type::ERROR;
    }
    
    stepperControl->initBus(boardAddrs[0]);

    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> StepperDriver::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> StepperDriver::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    return command_interfaces;
}

hardware_interface::return_type StepperDriver::start()
{
    for (uint i = 0; i < this->hw_states_.size(); i++) {
        if (std::isnan(this->hw_states_[i])) {
            this->hw_states_[i] = 0;
            this->hw_commands_[i] = 0;
        }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("StepperDriver"),
        "motor driver sucessfully started!");

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepperDriver::stop()
{
    stepperControl->holdPosition();

    RCLCPP_INFO(
        rclcpp::get_logger("StepperDriver"),
        "motor driver sucessfully stopped!");

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepperDriver::read()
{
    RCLCPP_INFO(
        rclcpp::get_logger("StepperDriver"),
        "reading...");

    for (uint i = 0; i < hw_states_.size(); i++) {
        RCLCPP_INFO(
            rclcpp::get_logger("StepperDriver"),
            "got state %.5f for joint %d!", hw_states_[i], i);
    }

    RCLCPP_INFO(
        rclcpp::get_logger("StepperDriver"),
        "joints sucessfully read!");

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepperDriver::write()
{
    RCLCPP_INFO(
        rclcpp::get_logger("StepperDriver"),
        "writing...");

    for (uint i = 0; i < hw_commands_.size(); i++) {
        RCLCPP_INFO(
            rclcpp::get_logger("StepperDriver"),
            "got command %.5f for joint %d!", hw_commands_[i], i);
    }

    bool backwards = hw_commands_[0] < 0;

    for (int i = 0; i < std::abs(hw_commands_[0]); i++)
    {
        this->stepperControl->move(backwards);
        hw_states_[0] += backwards ? -1 : 1;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("StepperDriver"),
        "joints sucessfully written!");

    return hardware_interface::return_type::OK;
}

}