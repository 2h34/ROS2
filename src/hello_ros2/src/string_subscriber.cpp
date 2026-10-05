#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class StringSubscriber : public rclcpp::Node
{
public:
    StringSubscriber()
    : Node("string_subscriber")
    {
        subscription_ =
            this->create_subscription<std_msgs::msg::String>(
                "day3_chatter",
                10,
                std::bind(
                    &StringSubscriber::topic_callback,
                    this,
                    std::placeholders::_1));
    }

private:
    void topic_callback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received: '%s'",
            msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StringSubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}