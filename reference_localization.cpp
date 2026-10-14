
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

// 根据 (x, y, yaw) 创建二维位姿变换
Eigen::Isometry2d makePose(double x, double y, double yaw)
{
    Eigen::Isometry2d T = Eigen::Isometry2d::Identity();

    T.linear() = Eigen::Rotation2Dd(yaw).toRotationMatrix();
    T.translation() = Eigen::Vector2d(x, y);

    return T;
}

int main()
{
    const double pi = std::acos(-1.0);

    // 已知参考物在地图中的位姿（测试数据）
    auto T_map_box = makePose(4.0, 3.0, pi / 2.0);

    // LiDAR 对参考物的相对观测（测试数据）
    auto T_laser_box = makePose(2.0, 0.0, 0.0);

    // 根据参考物反推 LiDAR 位姿
    auto T_map_laser = T_map_box * T_laser_box.inverse();

    // 提取定位结果
    auto position = T_map_laser.translation();

    double yaw = std::atan2(
        T_map_laser.linear()(1, 0),
        T_map_laser.linear()(0, 0)
    );

    std::cout << "x = " << position.x() << '\n';
    std::cout << "y = " << position.y() << '\n';
    std::cout << "yaw = " << yaw * 180.0 / pi << " deg\n";

    // 往返一致性验证
    bool valid = (T_map_laser * T_laser_box)
        .matrix().isApprox(T_map_box.matrix(), 1e-9);

    std::cout << "transform check = " << valid << '\n';

    return valid ? 0 : 1;
}
