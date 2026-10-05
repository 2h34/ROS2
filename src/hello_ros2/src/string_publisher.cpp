#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class StringPublisher : public rclcpp::Node
{
public:
    StringPublisher()
    : Node("robot_sender")
    {
        publisher_ =
            this->create_publisher<std_msgs::msg::String>(
                "day3_chatter",
                10);

        timer_ =
            this->create_wall_timer(
                1s,
                std::bind(
                    &StringPublisher::timer_callback,
                    this));
        counter_ = 0;
    }

private:
    void timer_callback()
    {
        std_msgs::msg::String message;

        counter_++;
        message.data = "Hello from C++ publisher: " + std::to_string(counter_);

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: '%s'",
            message.data.c_str());

        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int counter_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StringPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}