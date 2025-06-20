#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class EmergencyStop : public rclcpp::Node
{
public:
    EmergencyStop()
    : Node("emergency_stop")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/proximity_warning", 10, std::bind(&EmergencyStop::warning_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
private:
    void warning_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "STOP") {
            RCLCPP_INFO(this->get_logger(), "Emergency stop triggered.");

            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            publisher_->publish(twist);
        }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyStop>());
    rclcpp::shutdown();
    return 0;
}
