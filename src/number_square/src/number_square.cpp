#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class NumberSubscriber : public rclcpp::Node {
public:
    NumberSubscriber() : Node("number_subscriber") 
    {
        sub_1 = this->create_subscription<std_msgs::msg::Int32>(
            "sending_nums", 10,
            std::bind(&NumberSubscriber::callback_1, this, std::placeholders::_1));

        sub_2 = this->create_subscription<std_msgs::msg::Int32>(                        //just to show more than one sub can be subbed to one topic
            "sending_nums", 10,
            std::bind(&NumberSubscriber::callback_2, this, std::placeholders::_1));
    }

private:
    void callback_1(const std_msgs::msg::Int32::SharedPtr msg) 
    {
        int square = msg->data * msg->data;
        RCLCPP_INFO(this->get_logger(), "Received: %d, Square: %d", msg->data, square);
    }

    void callback_2(const std_msgs::msg::Int32::SharedPtr msg) 
    {
        int square = msg->data * msg->data * msg->data;
        RCLCPP_INFO(this->get_logger(), "Received: %d, Cube: %d", msg->data, square);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_1;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_2;

};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberSubscriber>());
    rclcpp::shutdown();
    return 0;
}