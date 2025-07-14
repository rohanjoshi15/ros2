// Detect orange cones and publish their center x-coordinate as geometry_msgs::msg::Point.z=1 if visible

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ConeDetector : public rclcpp::Node {
public:
    ConeDetector() : Node("cone_detector") {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ConeDetector::image_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::Point>("/cone_position", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame, hsv, mask;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (...) { return; }

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(30, 255, 255), mask);

        // Morphological cleanup
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        geometry_msgs::msg::Point cone_msg;
        cone_msg.z = 0.0;
        for (const auto &c : contours) {
            if (cv::contourArea(c) > 300) {
                auto rect = cv::boundingRect(c);
                cone_msg.x = rect.x + rect.width / 2;
                cone_msg.y = rect.y + rect.height / 2;
                cone_msg.z = 1.0;

                // Draw bounding box
                cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2);
                break;
            }
        }

        pub_->publish(cone_msg);

        // Show output windows
        cv::imshow("Masked Orange Areas", mask);
        cv::imshow("Camera with Bounding Box", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeDetector>());
    rclcpp::shutdown();
    return 0;
}