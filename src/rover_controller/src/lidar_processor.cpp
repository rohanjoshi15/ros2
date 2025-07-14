// Publishes minimum LiDAR range to /min_distance

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

class LidarProcessor : public rclcpp::Node {
public:
    LidarProcessor() : Node("lidar_processor") {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarProcessor::scan_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<std_msgs::msg::Float32>("/min_distance", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float min_val = std::numeric_limits<float>::infinity();
        for (auto r : msg->ranges)
            if (std::isfinite(r) && r < min_val)
                min_val = r;
        std_msgs::msg::Float32 out;
        out.data = min_val;
        pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}
