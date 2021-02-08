#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int32.hpp>

namespace passbutter { class StepperControl; }

namespace passbutter_driver
{

class SingleStepperDriverNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr _steps;
    void stepperCallback(const example_interfaces::msg::Int32::SharedPtr msg);

    passbutter::StepperControl *stepperControl;
    bool foundBoard;

public:
    SingleStepperDriverNode();
};

}