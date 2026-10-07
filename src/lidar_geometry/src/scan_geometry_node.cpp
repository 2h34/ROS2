#include <functional>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

struct Point2D
{
    double x;
    double y;
    std::size_t scan_index;
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
            point.scan_index = i;

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

        const double x_min = -0.20;
        const double x_max = 0.25;
        const double y_min = -0.34;
        const double y_max = -0.25;

        std::vector<Point2D> candidate_points;

        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const Point2D point = points[i];

            if (point.x >= x_min &&
                point.x <= x_max &&
                point.y >= y_min &&
                point.y <= y_max)
            {
                candidate_points.push_back(point);
            }
        }

        if (!candidate_points.empty())
        {
            /*求解是否有明显跳跃,计算有几段点*/
            std::size_t max_index_gap = 0;
            double max_point_gap = 0.0;
            std::size_t max_gap_prev_index = 0;
            std::size_t max_gap_curr_index = 0;

            Point2D max_gap_prev_point;
            Point2D max_gap_curr_point;

            const double gap_threshold = 0.02;

            std::vector<std::size_t> segment_sizes;

            std::size_t current_segment_size = 1;

            for (std::size_t i = 1; i < candidate_points.size(); ++i)
            {
                const std::size_t index_gap =
                    candidate_points[i].scan_index -
                    candidate_points[i - 1].scan_index;

                const double dx =
                    candidate_points[i].x -
                    candidate_points[i - 1].x;

                const double dy =
                    candidate_points[i].y -
                    candidate_points[i - 1].y;

                const double point_gap =
                    std::hypot(dx, dy); /*计算点间距*/

                if (index_gap > max_index_gap)
                {
                    max_index_gap = index_gap;
                }

                if (point_gap > max_point_gap)
                {
                    max_point_gap = point_gap;

                    max_gap_prev_index =
                        candidate_points[i - 1].scan_index;

                    max_gap_curr_index =
                        candidate_points[i].scan_index;

                    max_gap_prev_point =
                        candidate_points[i - 1];

                    max_gap_curr_point =
                        candidate_points[i];
                }
                if (point_gap > gap_threshold)
                {
                    segment_sizes.push_back(current_segment_size);
                    current_segment_size = 1;
                }
                else
                {
                    ++current_segment_size;
                }
            }
            segment_sizes.push_back(current_segment_size); /*最后一段*/
            RCLCPP_INFO(
                this->get_logger(),
                "segments: count=%zu",
                segment_sizes.size());
            for (std::size_t i = 0; i < segment_sizes.size(); ++i)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "segment[%zu]: size=%zu",
                    i,
                    segment_sizes[i]);
            }
            RCLCPP_INFO(
                this->get_logger(),
                "continuity: scan_index=[%zu, %zu] "
                "max_index_gap=%zu max_point_gap=%.3f m",
                candidate_points.front().scan_index,
                candidate_points.back().scan_index,
                max_index_gap,
                max_point_gap);
            RCLCPP_INFO(
                this->get_logger(),
                "max_gap: scan=%zu->%zu "
                "p1=(%.3f, %.3f) p2=(%.3f, %.3f) gap=%.3f m",
                max_gap_prev_index,
                max_gap_curr_index,
                max_gap_prev_point.x,
                max_gap_prev_point.y,
                max_gap_curr_point.x,
                max_gap_curr_point.y,
                max_point_gap);

            /*求解质心*/
            double sum_x = 0.0;
            double sum_y = 0.0;

            for (std::size_t i = 0; i < candidate_points.size(); ++i)
            {
                sum_x += candidate_points[i].x;
                sum_y += candidate_points[i].y;
            }

            const double centroid_x =
                sum_x / candidate_points.size();

            const double centroid_y =
                sum_y / candidate_points.size();

            /*求解协方差矩阵*/
            double s_xx = 0.0;
            double s_xy = 0.0;
            double s_yy = 0.0;

            for (std::size_t i = 0; i < candidate_points.size(); ++i)
            {
                const double dx =
                    candidate_points[i].x - centroid_x;

                const double dy =
                    candidate_points[i].y - centroid_y;

                s_xx += dx * dx;
                s_xy += dx * dy;
                s_yy += dy * dy;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "scatter: Sxx=%.6f Sxy=%.6f Syy=%.6f",
                s_xx,
                s_xy,
                s_yy);

            /*求解特征值，特征向量*/
            Eigen::Matrix2d scatter;
            scatter << s_xx, s_xy,
                s_xy, s_yy;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(scatter);
            const double lambda_min =
                solver.eigenvalues()(0);
            const double lambda_max =
                solver.eigenvalues()(1);
            const Eigen::Vector2d normal =
                solver.eigenvectors().col(0);
            const Eigen::Vector2d direction =
                solver.eigenvectors().col(1);

            RCLCPP_INFO(
                this->get_logger(),
                "eigen: lambda_min=%.6f lambda_max=%.6f "
                "normal=(%.4f, %.4f) direction=(%.4f, %.4f)",
                lambda_min,
                lambda_max,
                normal.x(),
                normal.y(),
                direction.x(),
                direction.y());

            /*求解直线方程*/
            const double a = normal.x();
            const double b = normal.y();
            const double c =
                -(a * centroid_x + b * centroid_y);
            RCLCPP_INFO(
                this->get_logger(),
                "line: %.6f*x + %.6f*y + %.6f = 0",
                a,
                b,
                c);
            const double centroid_residual =
                a * centroid_x +
                b * centroid_y +
                c;
            RCLCPP_INFO(
                this->get_logger(),
                "centroid_residual=%.9f",
                centroid_residual);

            /*求解距离，方向，可见长度, 可见中点*/
            const double distance =
                std::abs(c);
            const double direction_angle_rad =
                std::atan2(
                    direction.y(),
                    direction.x());
            const double direction_angle_deg =
                direction_angle_rad *
                180.0 /
                3.14159265358979323846;
            const double first_dx =
                candidate_points[0].x - centroid_x;
            const double first_dy =
                candidate_points[0].y - centroid_y;
            const double first_projection =
                first_dx * direction.x() +
                first_dy * direction.y();
            double projection_min = first_projection;
            double projection_max = first_projection;
            for (std::size_t i = 1; i < candidate_points.size(); ++i)
            {
                const double dx =
                    candidate_points[i].x - centroid_x;

                const double dy =
                    candidate_points[i].y - centroid_y;

                const double projection =
                    dx * direction.x() +
                    dy * direction.y();

                if (projection < projection_min)
                {
                    projection_min = projection;
                }

                if (projection > projection_max)
                {
                    projection_max = projection;
                }
            }
            const double visible_length =
                projection_max - projection_min;
            const double projection_mid =
                (projection_min + projection_max) / 2.0;
            const double visible_mid_x =
                centroid_x +
                projection_mid * direction.x();
            const double visible_mid_y =
                centroid_y +
                projection_mid * direction.y();

            const double endpoint_min_x =
                centroid_x +
                projection_min * direction.x();

            const double endpoint_min_y =
                centroid_y +
                projection_min * direction.y();

            const double endpoint_max_x =
                centroid_x +
                projection_max * direction.x();

            const double endpoint_max_y =
                centroid_y +
                projection_max * direction.y();

            RCLCPP_INFO(
                this->get_logger(),
                "endpoints: p_min=(%.3f, %.3f) p_max=(%.3f, %.3f)",
                endpoint_min_x,
                endpoint_min_y,
                endpoint_max_x,
                endpoint_max_y);
            RCLCPP_INFO(
                this->get_logger(),
                "geometry: distance=%.3f m angle=%.2f deg visible_length=%.3f m",
                distance,
                direction_angle_deg,
                visible_length);
            RCLCPP_INFO(
                this->get_logger(),
                "visible_mid: s_mid=%.4f m x=%.3f y=%.3f",
                projection_mid,
                visible_mid_x,
                visible_mid_y);

            /*求解实际的x，y范围*/
            double candidate_x_min = candidate_points[0].x;
            double candidate_x_max = candidate_points[0].x;
            double candidate_y_min = candidate_points[0].y;
            double candidate_y_max = candidate_points[0].y;

            RCLCPP_INFO(
                this->get_logger(),
                "centroid: x=%.3f y=%.3f",
                centroid_x,
                centroid_y);

            for (std::size_t i = 1; i < candidate_points.size(); ++i)
            {
                const Point2D point = candidate_points[i];

                if (point.x < candidate_x_min)
                {
                    candidate_x_min = point.x;
                }

                if (point.x > candidate_x_max)
                {
                    candidate_x_max = point.x;
                }

                if (point.y < candidate_y_min)
                {
                    candidate_y_min = point.y;
                }

                if (point.y > candidate_y_max)
                {
                    candidate_y_max = point.y;
                }
            }

            RCLCPP_INFO(
                this->get_logger(),
                "candidate_range: x=[%.3f, %.3f] y=[%.3f, %.3f]",
                candidate_x_min,
                candidate_x_max,
                candidate_y_min,
                candidate_y_max);

            /*求显示平均距离偏移量，并校准角度范围*/
            const double fit_rmse =
                std::sqrt(
                    lambda_min /
                    static_cast<double>(candidate_points.size()));
            const double pi = 3.14159265358979323846;

            double face_yaw_rad =
                std::atan2(
                    direction.y(),
                    direction.x());

            if (face_yaw_rad >= pi / 2.0)
            {
                face_yaw_rad -= pi;
            }
            else if (face_yaw_rad < -pi / 2.0)
            {
                face_yaw_rad += pi;
            }

            const double face_yaw_deg =
                face_yaw_rad * 180.0 / pi;
            RCLCPP_INFO(
                this->get_logger(),
                "observation: "
                "mid=(%.3f, %.3f) "
                "distance=%.3f m "
                "yaw=%.2f deg "
                "width=%.3f m "
                "fit_rmse=%.4f m",
                visible_mid_x,
                visible_mid_y,
                distance,
                face_yaw_deg,
                visible_length,
                fit_rmse);

            /*计算底盘误差*/
            const double target_mid_x = 0.0;
            const double target_distance = 0.30; /*Test,实际来自真实任务*/
            const double target_yaw_rad = 0.0;

            const double error_x =
                visible_mid_x - target_mid_x;

            const double error_distance =
                distance - target_distance;

            double error_yaw =
                face_yaw_rad - target_yaw_rad;

            if (error_yaw >= pi / 2.0)
            {
                error_yaw -= pi;
            }
            else if (error_yaw < -pi / 2.0)
            {
                error_yaw += pi;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "error: x=%.3f  distance=%.3f  yaw=%.2f deg",
                error_x,
                error_distance,
                error_yaw * 180.0 / pi);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "raw=%zu valid=%zu rejected=%zu points=%zu candidate=%zu",
            raw_count,
            valid_count,
            rejected_count,
            points.size(),
            candidate_points.size());
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