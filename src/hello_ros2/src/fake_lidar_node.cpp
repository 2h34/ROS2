#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class FakeLidarNode : public rclcpp::Node
{
public:
    FakeLidarNode()
    : Node("fake_lidar")
    {
        this->declare_parameter<std::string>("frame_id", "laser");
        frame_id_ = this->get_parameter("frame_id").as_string();

        this->declare_parameter<double>("range_min", 0.1);
        range_min_ = this->get_parameter("range_min").as_double();

        this->declare_parameter<double>("range_max", 10.0);
        range_max_ = this->get_parameter("range_max").as_double();

        this->declare_parameter<double>("publish_rate_hz", 2.0);
        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

        publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/scan",rclcpp::SensorDataQoS());

        auto publish_period =std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ =
            this->create_wall_timer(
                publish_period,
                std::bind(&FakeLidarNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        sensor_msgs::msg::LaserScan message;

        // Header
        message.header.stamp = this->now();
        message.header.frame_id = frame_id_;

        // 扫描角度
        message.angle_min = -1.0;
        message.angle_max = 1.0;
        message.angle_increment = 0.5;

        // 扫描时间
        
        message.scan_time = 1.0 / publish_rate_hz_;
        message.time_increment = message.scan_time / 4.0;

        // 有效测距范围
        message.range_min = range_min_;
        message.range_max = range_max_;

        // 5 个方向上的模拟距离
        message.ranges = {
            1.2,
            0.8,
            2.0,
            1.5,
            3.1
        };

        publisher_->publish(message);

        RCLCPP_INFO(
            this->get_logger(),
            "Published fake LaserScan with %zu ranges",
            message.ranges.size());
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id_;
    double range_min_;
    double range_max_;
    double publish_rate_hz_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FakeLidarNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}