#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

struct Point2D
{
    double x;
    double y;
};

class ScanGeometryNode : public rclcpp::Node
{
public:
    ScanGeometryNode()
        : Node("scan_geometry_node")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();

        subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                qos,
                std::bind(
                    &ScanGeometryNode::scan_callback,
                    this,
                    std::placeholders::_1));
    }

private:
    void scan_callback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const std::size_t raw_count = msg->ranges.size();

        std::size_t valid_count = 0;
        std::size_t rejected_count = 0;
        std::vector<Point2D> points;

        for (std::size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const float r = msg->ranges[i];

            if (!std::isfinite(r))
            {
                ++rejected_count;
                continue;
            }

            if (r < msg->range_min || r > msg->range_max)
            {
                ++rejected_count;
                continue;
            }

            // const double theta =
            //     msg->angle_min + i * msg->angle_increment;

            // Point2D point;
            // point.x = r * std::cos(theta);
            // point.y = r * std::sin(theta);

            // points.push_back(point);
            // ++valid_count;
            const double theta =
                msg->angle_min + i * msg->angle_increment;

            Point2D point;
            point.x = r * std::cos(theta);
            point.y = r * std::sin(theta);

            if (valid_count == 0)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "sample: i=%zu r=%.3f theta=%.3f x=%.3f y=%.3f",
                    i,
                    r,
                    theta,
                    point.x,
                    point.y);
            }

            points.push_back(point);

            ++valid_count;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "raw=%zu valid=%zu rejected=%zu points=%zu",
            raw_count,
            valid_count,
            rejected_count,
            points.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ScanGeometryNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}