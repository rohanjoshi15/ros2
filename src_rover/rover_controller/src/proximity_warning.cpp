#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class ProximityWarning : public rclcpp::Node
{
public:
    ProximityWarning()
    : Node("proximity_warning")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/min_distance", 10, std::bind(&ProximityWarning::distance_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::String>("/proximity_warning", 10);
    }
private:
    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (msg->data <= 0.9) {
            RCLCPP_WARN(this->get_logger(), "Wall is close!");
            std_msgs::msg::String warning;
            warning.data = "STOP";
            publisher_->publish(warning);
        }
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProximityWarning>());
    rclcpp::shutdown();
    return 0;
}
