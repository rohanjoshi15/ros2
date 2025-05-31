#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class NumberSubscriber : public rclcpp::Node {
public:

//We can give different QOS Settingds to different subscribers , subscribed to the same topic, like how i have shown here rn
    NumberSubscriber() : Node("number_subscriber") 
    {
        // so ive given Reliable and transient local for sub_1
        rclcpp::QoS qos_1(rclcpp::KeepLast(10));
        qos_1
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        // and here beest effort and volatile for sub_2
        rclcpp::QoS qos_2(rclcpp::KeepLast(5));
        qos_2
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        sub_1 = this->create_subscription<std_msgs::msg::Int32>(
            "sending_nums", qos_1,
            std::bind(&NumberSubscriber::callback_1, this, std::placeholders::_1));

        sub_2 = this->create_subscription<std_msgs::msg::Int32>(
            "sending_nums", qos_2,
            std::bind(&NumberSubscriber::callback_2, this, std::placeholders::_1));
    }

private:
    void callback_1(const std_msgs::msg::Int32::SharedPtr msg) 
    {
        int square = msg->data * msg->data;
        RCLCPP_INFO(this->get_logger(), "[QoS1] Received: %d, Square: %d", msg->data, square);
    }

    void callback_2(const std_msgs::msg::Int32::SharedPtr msg) 
    {
        int cube = msg->data * msg->data * msg->data;
        RCLCPP_INFO(this->get_logger(), "[QoS2] Received: %d, Cube: %d", msg->data, cube);
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
