#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int32.hpp>

namespace passbutter_driver
{

class SingleStepperCommandNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr _steps;
    rclcpp::TimerBase::SharedPtr _timer;

    int _stepCount;
    bool _reverse;

    void timer_callback();

public:
    SingleStepperCommandNode();
    
};

}