#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float32.hpp>
#include <vector>

namespace passbutter { class MotorControl; }

namespace passbutter_driver
{

class WheelDriverNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr _speed;
    void wheelCallback(const example_interfaces::msg::Float32::SharedPtr msg);

    std::vector<passbutter::MotorControl*> wheelControls;

public:
    WheelDriverNode();
    ~WheelDriverNode();
};

}