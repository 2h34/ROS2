#include "rclcpp/rclcpp.hpp"

class HelloNode : public rclcpp::Node
{
    public:
        HelloNode() : Node("hello_node")
        {
            RCLCPP_INFO(this->get_logger(), "Hello ROS2!");
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}