#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int32.hpp>
#include <vector>

namespace passbutter { class StepperControl; }

namespace passbutter_driver
{

class StepperDriverNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr _steps;
    void stepperCallback(const example_interfaces::msg::Int32::SharedPtr msg);

    std::vector<passbutter::StepperControl*> stepperControls;
    int _stepDelayMS;

public:
    StepperDriverNode();
};

}