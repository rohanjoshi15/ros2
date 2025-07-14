// Main FSM node controlling rover behavior based on cone and obstacle distance

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;

enum State {
    SEARCHING,
    APPROACHING_CONE,
    STOPPED_AT_CONE,
    SPINNING_AT_CONE,
    STRAIGHT_MOVING,
    RANDOM_TURNING,
    AVOID_WALL
};

class RoverController : public rclcpp::Node {
public:
    RoverController() : Node("rover_controller"), state_(SEARCHING), distance_(100.0f) {
        cone_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/cone_position", 10,
            std::bind(&RoverController::cone_callback, this, std::placeholders::_1));
        lidar_sub_ = this->create_subscription<std_msgs::msg::Float32>("/min_distance", 10,
            std::bind(&RoverController::lidar_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&RoverController::tick, this));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cone_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;
    float distance_;
    geometry_msgs::msg::Point last_cone_;
    rclcpp::Time spin_start_time_;
    rclcpp::Time straight_start_time_;

    void cone_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        last_cone_ = *msg;
    }

    void lidar_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        distance_ = msg->data;
    }

    void tick() {
        geometry_msgs::msg::Twist cmd;
        switch (state_) {
            case SEARCHING:
                if (last_cone_.z == 1.0) {
                    state_ = APPROACHING_CONE;
                } else {
                    cmd.angular.z = 0.4;
                }
                break;

            case APPROACHING_CONE:
                if (distance_ <= 4.0) {
                    cmd.linear.x = 0.0;
                    state_ = STOPPED_AT_CONE;
                } else {
                    float error = last_cone_.x - 320.0f;
                    cmd.linear.x = 0.4;
                    cmd.angular.z = -error * 0.002;
                }
                break;

            case STOPPED_AT_CONE:
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                spin_start_time_ = now();
                state_ = SPINNING_AT_CONE;
                break;

            case SPINNING_AT_CONE:
                if (last_cone_.z == 1.0 && distance_ > 4.0) {
                    state_ = APPROACHING_CONE;
                } else if ((now() - spin_start_time_).seconds() > 4.0) {
                    straight_start_time_ = now();
                    state_ = STRAIGHT_MOVING;
                } else {
                    cmd.angular.z = 0.5;
                }
                break;

            case STRAIGHT_MOVING:
                if (last_cone_.z == 1.0 && distance_ > 4.0) {
                    state_ = APPROACHING_CONE;
                } else if (distance_ <= 4.0) {
                    state_ = AVOID_WALL;
                } else if ((now() - straight_start_time_).seconds() > 3.0) {
                    state_ = RANDOM_TURNING;
                } else {
                    cmd.linear.x = 0.3;
                }
                break;

            case RANDOM_TURNING:
                cmd.angular.z = ((std::rand() % 2 == 0) ? 1.0 : -1.0);
                cmd.linear.x = 0.0;
                state_ = STRAIGHT_MOVING;
                straight_start_time_ = now();
                break;

            case AVOID_WALL:
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.5;
                if (distance_ > 4.5) {
                    state_ = STRAIGHT_MOVING;
                    straight_start_time_ = now();
                }
                break;
        }
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverController>());
    rclcpp::shutdown();
    return 0;
}
