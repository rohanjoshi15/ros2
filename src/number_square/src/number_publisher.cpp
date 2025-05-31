#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class NumberPublisher : public rclcpp::Node 
{
public:
    NumberPublisher() : Node("number_publisher"), num(1) 
    {
        // gonna give QOS Profiles here - 
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)// we can also giv best effort here 
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)// also can give transient local
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
            .keep_last(15) //here the number in the bracket is the Queue size, Queue size is only when the history is keep last
            .deadline(rclcpp::Duration::from_seconds(2))
            .lifespan(rclcpp::Duration::from_seconds(4))
            .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
            .liveliness_lease_duration(rclcpp::Duration::from_seconds(3));

        pub = this->create_publisher<std_msgs::msg::Int32>("sending_nums", qos_profile);

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
