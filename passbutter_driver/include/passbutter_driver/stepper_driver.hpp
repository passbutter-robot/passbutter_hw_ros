#pragma once

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include <vector>

namespace passbutter { class StepperControl; }

namespace passbutter_driver
{

class StepperDriver : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
    StepperDriver();
    ~StepperDriver();

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type start() override;
    hardware_interface::return_type stop() override;
    hardware_interface::return_type read() override;
    hardware_interface::return_type write() override;

private:
    passbutter::StepperControl *stepperControl;

    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
};

}