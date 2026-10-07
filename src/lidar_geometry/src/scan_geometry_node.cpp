#include <functional>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

struct Point2D
{
    double x;
    double y;
    std::size_t scan_index;
};

struct LineFit
{
    Eigen::Vector2d centroid;

    Eigen::Vector2d normal;
    Eigen::Vector2d direction;

    double lambda_min;
    double lambda_max;

    double c;

    double projection_min;
    double projection_max;

    Eigen::Vector2d endpoint_min;
    Eigen::Vector2d endpoint_max;
    Eigen::Vector2d visible_mid;

    double visible_length;
    double distance;
    double yaw_rad;
    double rmse;
};

LineFit fit_line_tls(const std::vector<Point2D> &points)
{
    LineFit fit;

    // 1. 计算质心
    double sum_x = 0.0;
    double sum_y = 0.0;

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        sum_x += points[i].x;
        sum_y += points[i].y;
    }

    fit.centroid.x() =
        sum_x / static_cast<double>(points.size());

    fit.centroid.y() =
        sum_y / static_cast<double>(points.size());

    // 2. 计算 scatter matrix
    double s_xx = 0.0;
    double s_xy = 0.0;
    double s_yy = 0.0;

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const double dx =
            points[i].x - fit.centroid.x();

        const double dy =
            points[i].y - fit.centroid.y();

        s_xx += dx * dx;
        s_xy += dx * dy;
        s_yy += dy * dy;
    }

    Eigen::Matrix2d scatter;
    scatter << s_xx, s_xy,
        s_xy, s_yy;

    // 3. 特征分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(scatter);

    fit.lambda_min =
        solver.eigenvalues()(0);

    fit.lambda_max =
        solver.eigenvalues()(1);

    fit.normal =
        solver.eigenvectors().col(0);

    fit.direction =
        solver.eigenvectors().col(1);

    // 4. 计算直线方程参数
    fit.c = -(fit.normal.x() * fit.centroid.x() +
              fit.normal.y() * fit.centroid.y());
    fit.distance =
        std::abs(fit.c);

    // 5. 计算投影范围和可见长度
    const double first_dx =
        points[0].x - fit.centroid.x();

    const double first_dy =
        points[0].y - fit.centroid.y();

    const double first_projection =
        first_dx * fit.direction.x() +
        first_dy * fit.direction.y();

    fit.projection_min = first_projection;
    fit.projection_max = first_projection;

    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const double dx =
            points[i].x - fit.centroid.x();

        const double dy =
            points[i].y - fit.centroid.y();

        const double projection =
            dx * fit.direction.x() +
            dy * fit.direction.y();

        if (projection < fit.projection_min)
        {
            fit.projection_min = projection;
        }

        if (projection > fit.projection_max)
        {
            fit.projection_max = projection;
        }
    }
    fit.visible_length =
        fit.projection_max - fit.projection_min;

    // 6. 计算可见中点和端点
    const double projection_mid =
        (fit.projection_min + fit.projection_max) / 2.0;

    fit.endpoint_min =
        fit.centroid +
        fit.projection_min * fit.direction;

    fit.endpoint_max =
        fit.centroid +
        fit.projection_max * fit.direction;

    fit.visible_mid =
        fit.centroid +
        projection_mid * fit.direction;

    // 7. 计算拟合误差
    fit.rmse =
        std::sqrt(
            fit.lambda_min /
            static_cast<double>(points.size()));

    // 8. 计算方向角度
    fit.yaw_rad = std::atan2(
        fit.direction.y(),
        fit.direction.x());
    const double pi = 3.14159265358979323846;
    if (fit.yaw_rad >= pi / 2.0)
    {
        fit.yaw_rad -= pi;
    }
    else if (fit.yaw_rad < -pi / 2.0)
    {
        fit.yaw_rad += pi;
    }

    return fit;
}

/// 将角度归一化到 [-pi/4, pi/4] 范围内
double canonicalize_square_yaw(double yaw_rad)
{
    const double pi =
        3.14159265358979323846;

    const double half_pi =
        pi / 2.0;

    const double quarter_pi =
        pi / 4.0;

    while (yaw_rad >= quarter_pi)
    {
        yaw_rad -= half_pi;
    }

    while (yaw_rad < -quarter_pi)
    {
        yaw_rad += half_pi;
    }

    return yaw_rad;
}

/// 计算两条直线方向向量的点积绝对值
double line_direction_dot(
    const LineFit &line1,
    const LineFit &line2)
{
    return std::abs(
        line1.direction.dot(line2.direction));
}

/// 计算两条直线的交点,如果平行则返回 false
bool intersect_lines(
    const LineFit &line1,
    const LineFit &line2,
    Eigen::Vector2d &intersection)
{
    const double a1 = line1.normal.x();
    const double b1 = line1.normal.y();

    const double a2 = line2.normal.x();
    const double b2 = line2.normal.y();

    const double determinant =
        a1 * b2 - a2 * b1;

    if (std::abs(determinant) < 1e-9)
    {
        return false;
    }

    intersection.x() =
        (b1 * line2.c - b2 * line1.c) /
        determinant;

    intersection.y() =
        (a2 * line1.c - a1 * line2.c) /
        determinant;

    return true;
}

/// 计算点到线段端点的最小距离
double distance_to_nearest_endpoint(
    const LineFit &line,
    const Eigen::Vector2d &point)
{
    const double d_min =
        (point - line.endpoint_min).norm();
    //.norm() 计算向量的长度。

    const double d_max =
        (point - line.endpoint_max).norm();

    return std::min(d_min, d_max);
}

/// 计算从角点到线段的边缘方向向量,判断是否反向
Eigen::Vector2d edge_direction_from_corner(
    const LineFit &line,
    const Eigen::Vector2d &corner)
{
    Eigen::Vector2d edge_direction =
        line.direction;

    const Eigen::Vector2d corner_to_segment =
        line.visible_mid - corner;

    if (edge_direction.dot(corner_to_segment) < 0.0)
    {
        edge_direction = -edge_direction;
    }

    return edge_direction;
}

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
        const double pi = 3.14159265358979323846;

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

        const double x_min = -0.25;
        const double x_max = 0.29;
        const double y_min = -1.10;
        const double y_max = -0.24;

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
            std::vector<std::vector<Point2D>> segments;

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

            const LineFit fit =
                fit_line_tls(candidate_points);
            const double box_side_length = 0.350;

            // 计算传感器原点到拟合线的法向量方向,以确定箱体中心位置
            const Eigen::Vector2d sensor_origin(0.0, 0.0);

            const Eigen::Vector2d to_sensor =
                sensor_origin - fit.visible_mid;

            Eigen::Vector2d inward_normal =
                fit.normal;

            if (fit.normal.dot(to_sensor) > 0.0)
            {
                inward_normal = -fit.normal;
            }
            const Eigen::Vector2d box_center =
                fit.visible_mid +
                0.5 * box_side_length * inward_normal;

            // 计算箱体中心点到传感器原点的方向角度,以确定箱体朝向
            const double box_yaw_rad =
                canonicalize_square_yaw(fit.yaw_rad);
            const double length_ratio =
                fit.visible_length / box_side_length;

            // splikt两条线段
            const std::size_t min_points_for_fit = 3;

            double best_objective =
                std::numeric_limits<double>::infinity(); // 初始化无穷大

            std::size_t best_split = 0;

            for (std::size_t k = min_points_for_fit;
                 k + min_points_for_fit <= candidate_points.size();
                 ++k)
            {
                std::vector<Point2D> points1(
                    candidate_points.begin(),
                    candidate_points.begin() + k);

                std::vector<Point2D> points2(
                    candidate_points.begin() + k,
                    candidate_points.end());

                const LineFit line1 =
                    fit_line_tls(points1);

                const LineFit line2 =
                    fit_line_tls(points2);

                const double objective =
                    line1.lambda_min +
                    line2.lambda_min;

                if (objective < best_objective)
                {
                    best_objective = objective;
                    best_split = k;
                }
            }
            RCLCPP_INFO(
                this->get_logger(),
                "two_line_search: "
                "N=%zu best_split=%zu "
                "sizes=(%zu,%zu) "
                "J=%.6f",
                candidate_points.size(),
                best_split,
                best_split,
                candidate_points.size() - best_split,
                best_objective);

            // 恢复最佳两组点，并拟合两条直线，求交点
            if (best_split > 0)
            {
                std::vector<Point2D> best_points1(
                    candidate_points.begin(),
                    candidate_points.begin() + best_split);

                std::vector<Point2D> best_points2(
                    candidate_points.begin() + best_split,
                    candidate_points.end());

                const LineFit best_line1 =
                    fit_line_tls(best_points1);

                const LineFit best_line2 =
                    fit_line_tls(best_points2);

                // 计算两条直线的有效点数量和可见长度
                const std::size_t support_points1 =
                    best_points1.size();

                const std::size_t support_points2 =
                    best_points2.size();

                const double support_length1 =
                    best_line1.visible_length;

                const double support_length2 =
                    best_line2.visible_length;

                const std::size_t min_support_points =
                    std::min(
                        support_points1,
                        support_points2);

                const double min_support_length =
                    std::min(
                        support_length1,
                        support_length2);

                RCLCPP_INFO(
                    this->get_logger(),
                    "two_line_support: "
                    "points=(%zu,%zu) "
                    "min_points=%zu "
                    "length=(%.3f,%.3f) "
                    "min_length=%.3f "
                    "rmse=(%.4f,%.4f)",
                    support_points1,
                    support_points2,
                    min_support_points,
                    support_length1,
                    support_length2,
                    min_support_length,
                    best_line1.rmse,
                    best_line2.rmse);

                // 计算两条直线方向向量的点积绝对值
                const double direction_dot = line_direction_dot(
                    best_line1,
                    best_line2);
                Eigen::Vector2d intersection;

                // 计算两条直线的交点,如果平行则返回 false
                const bool has_intersection =
                    intersect_lines(
                        best_line1,
                        best_line2,
                        intersection);
                if (has_intersection)
                {
                    // 计算交点到两条线段端点的最小距离
                    const double endpoint_distance1 =
                        distance_to_nearest_endpoint(
                            best_line1,
                            intersection);

                    const double endpoint_distance2 =
                        distance_to_nearest_endpoint(
                            best_line2,
                            intersection);

                    // 求边缘方向向量（确定方向）,得到正方形中心
                    const Eigen::Vector2d u1 =
                        edge_direction_from_corner(
                            best_line1,
                            intersection);

                    const Eigen::Vector2d u2 =
                        edge_direction_from_corner(
                            best_line2,
                            intersection);

                    const Eigen::Vector2d corner_candidate_center =
                        intersection + 0.5 * box_side_length * u1 + 0.5 * box_side_length * u2;

                    // 计算角点的方向角度,归一化到 [-pi/4, pi/4] 范围内
                    const double corner_candidate_yaw_rad =
                        canonicalize_square_yaw(
                            std::atan2(
                                u1.y(),
                                u1.x()));

                    RCLCPP_INFO(
                        this->get_logger(),
                        "two_line_geometry: "
                        "dot=%.4f "
                        "intersection=(%.3f, %.3f) "
                        "end_dist=(%.3f, %.3f)",
                        direction_dot,
                        intersection.x(),
                        intersection.y(),
                        endpoint_distance1,
                        endpoint_distance2);
                    RCLCPP_INFO(
                        this->get_logger(),
                        "corner_candidate: "
                        "u1=(%.3f,%.3f) "
                        "u2=(%.3f,%.3f) "
                        "center=(%.3f,%.3f) "
                        "yaw=%.2f deg",
                        u1.x(),
                        u1.y(),
                        u2.x(),
                        u2.y(),
                        corner_candidate_center.x(),
                        corner_candidate_center.y(),
                        corner_candidate_yaw_rad * 180.0 / pi);
                }
                else
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "two_line_geometry: "
                        "dot=%.4f "
                        "intersection=NONE",
                        direction_dot);
                }
            }
            RCLCPP_INFO(
                this->get_logger(),
                "single_face: "
                "length_ratio=%.3f "
                "inward=(%.4f, %.4f) "
                "box_center=(%.3f, %.3f) "
                "box_yaw=%.2f deg",
                length_ratio,
                inward_normal.x(),
                inward_normal.y(),
                box_center.x(),
                box_center.y(),
                box_yaw_rad * 180.0 / pi);

            RCLCPP_INFO(
                this->get_logger(),
                "eigen: lambda_min=%.6f lambda_max=%.6f "
                "normal=(%.4f, %.4f) direction=(%.4f, %.4f)",
                fit.lambda_min,
                fit.lambda_max,
                fit.normal.x(),
                fit.normal.y(),
                fit.direction.x(),
                fit.direction.y());

            RCLCPP_INFO(
                this->get_logger(),
                "line: %.6f*x + %.6f*y + %.6f = 0",
                fit.normal.x(),
                fit.normal.y(),
                fit.c);

            RCLCPP_INFO(
                this->get_logger(),
                "endpoints: p_min=(%.3f, %.3f) p_max=(%.3f, %.3f)",
                fit.endpoint_min.x(),
                fit.endpoint_min.y(),
                fit.endpoint_max.x(),
                fit.endpoint_max.y());

            const double pi = 3.14159265358979323846;

            RCLCPP_INFO(
                this->get_logger(),
                "observation: "
                "mid=(%.3f, %.3f) "
                "distance=%.3f m "
                "yaw=%.2f deg "
                "width=%.3f m "
                "fit_rmse=%.4f m",
                fit.visible_mid.x(),
                fit.visible_mid.y(),
                fit.distance,
                fit.yaw_rad * 180.0 / pi,
                fit.visible_length,
                fit.rmse);

            /*计算底盘误差*/
            const double target_mid_x = 0.0;
            const double target_distance = 0.30; /*Test,实际来自真实任务*/
            const double target_yaw_rad = 0.0;

            const double error_x =
                fit.visible_mid.x() - target_mid_x;

            const double error_distance =
                fit.distance - target_distance;

            double error_yaw =
                fit.yaw_rad - target_yaw_rad;

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