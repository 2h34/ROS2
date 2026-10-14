
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TfQueryNode : public rclcpp::Node
{
public:
    TfQueryNode() : Node("tf_query_node")
    {
        // 1. 创建 TF 缓存
        buffer_ = std::make_unique<tf2_ros::Buffer>(
            this->get_clock());

        // 2. 创建监听器，接收 TF 并填充 Buffer
        listener_ =
            std::make_shared<tf2_ros::TransformListener>(*buffer_);

        // 3. 每秒查询一次
        timer_ = this->create_wall_timer(
            1s, [this]()
            { query_tf(); });
    }

private:
    void query_tf()
    {
        try
        {
            auto tf = buffer_->lookupTransform(
                "p5_map",
                "p5_laser",
                tf2::TimePointZero);

            RCLCPP_INFO(
                this->get_logger(),
                "x=%.2f, y=%.2f, z=%.2f",
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "TF query failed: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfQueryNode>());
    rclcpp::shutdown();
    return 0;
}
