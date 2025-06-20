#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor()
    : Node("lidar_processor")
    {
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarProcessor::scan_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/min_distance", 10);
    }
private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float min_val = std::numeric_limits<float>::infinity();

        for (auto range : msg->ranges) {
            if (std::isfinite(range) && range < min_val) {
                min_val = range;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Front Distance: %.2f m", min_val);
        
        std_msgs::msg::Float32 distance;
        distance.data = min_val;
        publisher_->publish(distance);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}
