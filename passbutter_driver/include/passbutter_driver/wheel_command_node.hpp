#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float32.hpp>

namespace passbutter_driver
{

class WheelCommandNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr _wheel_speed;
    rclcpp::TimerBase::SharedPtr _timer;

    double _target_speed;
    bool _reverse;

    void timer_callback();

public:
    WheelCommandNode();
    
};

}