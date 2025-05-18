#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class NumberPublisher : public rclcpp::Node 
{
public:
    NumberPublisher() : Node("number_publisher"), num(1) 
    {
        pub = this->create_publisher<std_msgs::msg::Int32>("sending_nums", 10);
        timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NumberPublisher::publish_number, this));
    }

private:
    void publish_number() 
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = num++;
        RCLCPP_INFO(this->get_logger(), "Publishing: %d", msg.data);
        pub->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
    int num;
};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberPublisher>());
    rclcpp::shutdown();
    return 0;
}