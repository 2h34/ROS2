#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanSubscriber : public rclcpp::Node
{
public:
    ScanSubscriber()
    : Node("scan_subscriber")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();

        subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                qos,
                std::bind(
                    &ScanSubscriber::scan_callback,
                    this,
                    std::placeholders::_1));
    }

private:
    void scan_callback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received scan: %zu ranges",
            msg->ranges.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ScanSubscriber>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}